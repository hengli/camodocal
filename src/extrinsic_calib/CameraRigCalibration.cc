#include "CameraRigCalibration.h"

#include <iomanip>
#include <iostream>

#include "../../library/bundle_adj/ReprojectionError.h"
#include "../../library/gpl/CameraEnums.h"
#include "../../library/gpl/EigenUtils.h"
#include "../../mapping/feature_tracker/FeatureTracker.h"
#include "../../visualization/overlay/GLOverlayExtended.h"

#include "CalibrationWindow.h"
#include "CameraRigBA.h"

namespace camodocal
{

bool CameraRigCalibration::mStop = false;

OdoCamThread::OdoCamThread(int nMotions, int cameraIdx,
                           AtomicData<cv::Mat>* image,
                           const CataCameraCalibration* cameraCalib,
                           SensorDataBuffer<OdometerPtr>& odometerBuffer,
                           SensorDataBuffer<OdometerPtr>& interpOdometerBuffer,
                           boost::mutex& odometerBufferMutex,
                           SensorDataBuffer<PosePtr>& gpsInsBuffer,
                           SensorDataBuffer<PosePtr>& interpGpsInsBuffer,
                           boost::mutex& gpsInsBufferMutex,
                           std::string& status,
                           cv::Mat& sketch,
                           bool& stop,
                           bool saveImages,
                           bool verbose)
 : mThread(0)
 , mCameraIdx(cameraIdx)
 , mRunning(false)
 , mImage(image)
 , mCameraCalib(cameraCalib)
 , mOdometerBuffer(odometerBuffer)
 , mInterpOdometerBuffer(interpOdometerBuffer)
 , mOdometerBufferMutex(odometerBufferMutex)
 , mGpsInsBuffer(gpsInsBuffer)
 , mInterpGpsInsBuffer(interpGpsInsBuffer)
 , mGpsInsBufferMutex(gpsInsBufferMutex)
 , mStatus(status)
 , mSketch(sketch)
 , kKeyFrameDistance(0.25)
 , kMinTrackLength(15)
 , kOdometerTimeout(4.0)
 , mStop(stop)
 , mSaveImages(saveImages)
{
    mOdoCamCalib.setVerbose(verbose);
    mOdoCamCalib.setMotionCount(nMotions);
}

OdoCamThread::~OdoCamThread()
{
    g_return_if_fail(mThread == 0);
}

int
OdoCamThread::cameraIdx(void) const
{
    return mCameraIdx;
}

const Eigen::Matrix4d&
OdoCamThread::camOdoTransform(void) const
{
    return mCamOdoTransform;
}

const std::vector<FrameSegment>&
OdoCamThread::frameSegments(void) const
{
    return mFrameSegments;
}

void
OdoCamThread::reprojectionError(double& minError, double& maxError, double& avgError) const
{
    minError = std::numeric_limits<double>::max();
    maxError = std::numeric_limits<double>::min();

    size_t count = 0;
    double totalError = 0.0;

    for (size_t segmentIdx = 0; segmentIdx < mFrameSegments.size(); ++segmentIdx)
    {
        const FrameSegment& segment = mFrameSegments.at(segmentIdx);

        for (size_t frameIdx = 0; frameIdx < segment.size(); ++frameIdx)
        {
            const FrameConstPtr& frame = segment.at(frameIdx);

            const std::vector<Point2DFeaturePtr>& features2D = frame->features2D();

            for (size_t i = 0; i < features2D.size(); ++i)
            {
                const Point2DFeatureConstPtr& feature2D = features2D.at(i);
                const Point3DFeatureConstPtr& feature3D = feature2D->feature3D();

                if (feature3D.get() == 0)
                {
                    continue;
                }

                CameraReprojectionError reprojErr(mCameraCalib->cameraParameters(), feature2D->keypoint().pt.x, feature2D->keypoint().pt.y);

                double residuals[2];
                reprojErr(frame->camera()->rotationData(), frame->camera()->translationData(),
                          feature3D->pointData(), residuals);

                double error = hypot(residuals[0], residuals[1]);

                if (minError > error)
                {
                    minError = error;
                }
                if (maxError < error)
                {
                    maxError = error;
                }
                totalError += error;
                ++count;
            }
        }
    }

    if (count == 0)
    {
        avgError = 0.0;
        minError = 0.0;
        maxError = 0.0;

        return;
    }

    avgError = totalError / count;
}

void
OdoCamThread::launch(void)
{
    mThread = Glib::Threads::Thread::create(sigc::mem_fun(*this, &OdoCamThread::threadFunction));
}

void
OdoCamThread::join(void)
{
    if (mRunning)
    {
        mThread->join();
    }
    mThread = 0;
}

bool
OdoCamThread::running(void) const
{
    return mRunning;
}

sigc::signal<void>&
OdoCamThread::signalFinished(void)
{
    return mSignalFinished;
}

void
OdoCamThread::threadFunction(void)
{
    mRunning = true;

    TemporalFeatureTracker tracker(mCameraCalib->cameraParameters(),
                                   SURF_DETECTOR, SURF_DESCRIPTOR, RATIO, false);
    tracker.setVerbose(OdometerCameraCalibration::getVerbose());

    uint64_t prevTimeStamp = 0;
    Eigen::Vector2d prevPos(0.0, 0.0);
    bool receivedOdometer = false;

    cv::Mat image;
    cv::Mat colorImage;

    int trackBreaks = 0;

    const cv::Mat& imageMask = mCameraCalib->cameraMask();

    std::vector<OdometerPtr> odometerPoses;

    std::ostringstream oss;
    oss << "swba" << mCameraIdx + 1;
    GLOverlayExtended overlay(oss.str(), VCharge::COORDINATE_FRAME_GLOBAL);

    bool halt = false;

    while (!halt && !mOdoCamCalib.motionsEnough())
    {
        usleep(1000);

        bool lockFrame = mImage->tryLockData();

        if (!lockFrame)
        {
            continue;
        }

        if (!mImage->available() || mImage->timeStamp() == prevTimeStamp)
        {
            mImage->unlockData();

            continue;
        }

        uint64_t timeStamp = mImage->timeStamp();

        mImage->data().copyTo(image);

        mImage->unlockData();

        double ts0 = timeInSeconds();

        if (image.channels() == 1)
        {
            cv::cvtColor(image, colorImage, CV_GRAY2BGR);
        }
        else
        {
            image.copyTo(colorImage);
        }

        double ts1 = timeInSeconds();

        // skip if current car position is too near previous position
        OdometerPtr currOdometer;

        if (!mOdometerBuffer.current(currOdometer))
        {
            std::cout << "# WARNING: No data in odometer buffer." << std::endl;
        }
        else if (!receivedOdometer)
        {
            prevPos = currOdometer->position();
            receivedOdometer = true;
        }
        else if ((currOdometer->position() - prevPos).norm() > kKeyFrameDistance)
        {
            Eigen::Vector2d currPos = currOdometer->position();

            FramePtr frame(new Frame);
            image.copyTo(frame->image());

            Eigen::Matrix3d R;
            Eigen::Vector3d t;
            bool camValid = tracker.addFrame(frame, imageMask, R, t);

            mOdometerBufferMutex.lock();

            OdometerPtr interpOdo;
            if (!mInterpOdometerBuffer.find(timeStamp, interpOdo))
            {
                double timeStart = timeInSeconds();
                while (!interpolateOdometer(mOdometerBuffer, timeStamp, interpOdo))
                {
                    if (timeInSeconds() - timeStart > kOdometerTimeout)
                    {
                        std::cout << "# ERROR: No odometer data for " << kOdometerTimeout << "s. Exiting..." << std::endl;
                        exit(1);
                    }

                    usleep(1000);
                }

                mInterpOdometerBuffer.push(timeStamp, interpOdo);
            }

            mOdometerBufferMutex.unlock();

            mGpsInsBufferMutex.lock();

            PosePtr interpGpsIns;
            if (!mGpsInsBuffer.empty() && !mInterpGpsInsBuffer.find(timeStamp, interpGpsIns))
            {
                double timeStart = timeInSeconds();
                while (!interpolatePose(mGpsInsBuffer, timeStamp, interpGpsIns))
                {
                    if (timeInSeconds() - timeStart > kOdometerTimeout)
                    {
                        std::cout << "# ERROR: No GPS/INS data for " << kOdometerTimeout << "s. Exiting..." << std::endl;
                        exit(1);
                    }

                    usleep(1000);
                }

                mInterpGpsInsBuffer.push(timeStamp, interpGpsIns);
            }

            mGpsInsBufferMutex.unlock();

            // tag frame with odometer and GPS/INS data
            frame->odometer() = interpOdo;

            if (interpGpsIns.get() != 0)
            {
                frame->gps_ins() = interpGpsIns;
            }

            if (!mSaveImages)
            {
                frame->image() = cv::Mat();
            }

            if (camValid)
            {
                odometerPoses.push_back(interpOdo);

                prevPos = currPos;
            }

            if (mStop || !camValid ||
                (odometerPoses.size() >= kMinTrackLength &&
                 mOdoCamCalib.getCurrentMotionCount() + odometerPoses.size() > mOdoCamCalib.getMotionCount()))
            {
                addOdoCamCalibData(odometerPoses, tracker.getPoses(), tracker.getFrames());

                if (!odometerPoses.empty())
                {
                    odometerPoses.erase(odometerPoses.begin(), odometerPoses.begin() + odometerPoses.size() - 1);
                }

                ++trackBreaks;

                if (mStop)
                {
                    halt = true;
                }
            }
            else
            {
                // visualize camera poses and 3D scene points
                const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> >& poses = tracker.getPoses();

                overlay.clear();
                overlay.pointSize(2.0f);
                overlay.lineWidth(1.0f);

                // draw 3D scene points
//                switch (cameraIdx)
//                {
//                case CAMERA_FRONT:
//                    overlay.color4f(1.0f, 0.0f, 0.0f, 0.5f);
//                    break;
//                case CAMERA_LEFT:
//                    overlay.color4f(0.0f, 1.0f, 0.0f, 0.5f);
//                    break;
//                case CAMERA_REAR:
//                    overlay.color4f(0.0f, 0.0f, 1.0f, 0.5f);
//                    break;
//                case CAMERA_RIGHT:
//                    overlay.color4f(1.0f, 1.0f, 0.0f, 0.5f);
//                    break;
//                default:
//                    overlay.color4f(1.0f, 1.0f, 1.0f, 0.5f);
//                }
//
//                overlay.begin(VCharge::POINTS);
//
//                std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > scenePoints = tracker.getScenePoints();
//                for (size_t j = 0; j < scenePoints.size(); ++j)
//                {
//                    Eigen::Vector3d& p = scenePoints.at(j);
//
//                    overlay.vertex3f(p(2), -p(0), -p(1));
//                }
//
//                overlay.end();

                // draw cameras
                for (size_t j = 0; j < poses.size(); ++j)
                {
                    Eigen::Matrix4d H = poses.at(j).inverse();

                    double xBound = 0.1;
                    double yBound = 0.1;
                    double zFar = 0.2;

                    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > frustum;
                    frustum.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
                    frustum.push_back(Eigen::Vector3d(-xBound, -yBound, zFar));
                    frustum.push_back(Eigen::Vector3d(xBound, -yBound, zFar));
                    frustum.push_back(Eigen::Vector3d(xBound, yBound, zFar));
                    frustum.push_back(Eigen::Vector3d(-xBound, yBound, zFar));

                    for (size_t k = 0; k < frustum.size(); ++k)
                    {
                        frustum.at(k) = H.block<3,3>(0,0) * frustum.at(k) + H.block<3,1>(0,3);
                    }

                    overlay.color4f(1.0f, 1.0f, 1.0f, 1.0f);
                    overlay.begin(VCharge::LINES);

                    for (int k = 1; k < 5; ++k)
                    {
                        overlay.vertex3f(frustum.at(0)(2), -frustum.at(0)(0), -frustum.at(0)(1));
                        overlay.vertex3f(frustum.at(k)(2), -frustum.at(k)(0), -frustum.at(k)(1));
                    }

                    overlay.end();

                    switch (mCameraIdx)
                    {
                    case CAMERA_FRONT:
                        overlay.color4f(1.0f, 0.0f, 0.0f, 0.5f);
                        break;
                    case CAMERA_LEFT:
                        overlay.color4f(0.0f, 1.0f, 0.0f, 0.5f);
                        break;
                    case CAMERA_REAR:
                        overlay.color4f(0.0f, 0.0f, 1.0f, 0.5f);
                        break;
                    case CAMERA_RIGHT:
                        overlay.color4f(1.0f, 1.0f, 0.0f, 0.5f);
                        break;
                    default:
                        overlay.color4f(1.0f, 1.0f, 1.0f, 0.5f);
                    }

                    overlay.begin(VCharge::POLYGON);

                    for (int k = 1; k < 5; ++k)
                    {
                        overlay.vertex3f(frustum.at(k)(2), -frustum.at(k)(0), -frustum.at(k)(1));
                    }

                    overlay.end();
                }

                overlay.publish();
            }

            prevTimeStamp = timeStamp;
        }

        int currentMotionCount = 0;
        if (odometerPoses.size() >= kMinTrackLength)
        {
            currentMotionCount = odometerPoses.size() - 1;
        }

        std::ostringstream oss;
        oss << "# motions: " << mOdoCamCalib.getCurrentMotionCount() + currentMotionCount << " | "
            << "# track breaks: " << trackBreaks;

        CalibrationWindow::instance()->dataMutex().lock();

        mStatus.assign(oss.str());

        if (!tracker.getSketch().empty())
        {
            tracker.getSketch().copyTo(mSketch);
        }
        else
        {
            colorImage.copyTo(mSketch);
        }

        CalibrationWindow::instance()->dataMutex().unlock();
    }

    std::string buffer;
    std::string filename;
    switch (mCameraIdx)
    {
    case CAMERA_FRONT:
        buffer.assign("front ");
        filename.assign("odo_front_cam.txt");
        break;
    case CAMERA_LEFT:
        buffer.assign("left ");
        filename.assign("odo_left_cam.txt");
        break;
    case CAMERA_REAR:
        buffer.assign("rear ");
        filename.assign("odo_rear_cam.txt");
        break;
    case CAMERA_RIGHT:
        buffer.assign("right ");
        filename.assign("odo_right_cam.txt");
        break;
    }

    std::cout << "# INFO: Calibrating odometer - " + buffer + "camera..." << std::endl;

//    mOdoCamCalib.writeMotionSegmentsToFile(filename);

    Eigen::Matrix4d H_cam_odo;
    mOdoCamCalib.calibrate(H_cam_odo);

    std::cout << "# INFO: Finished calibrating odometer - " + buffer + "camera." << std::endl;
    std::cout << "Rotation: " << std::endl << H_cam_odo.block<3,3>(0,0) << std::endl;
    std::cout << "Translation: " << std::endl << H_cam_odo.block<3,1>(0,3).transpose() << std::endl;

    mCamOdoTransform = H_cam_odo;

    mRunning = false;

    mSignalFinished();
}

void
OdoCamThread::addOdoCamCalibData(const std::vector<OdometerPtr>& odoPoses,
                                 const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> >& camPoses,
                                 FrameSegment& frameSegment)
{
    if (odoPoses.size() != camPoses.size())
    {
        std::cout << "# WARNING: Numbers of odometer (" << odoPoses.size()
                  << ") and camera poses (" << camPoses.size() << ") differ. Aborting..." << std::endl;

        return;
    }

    if (odoPoses.size() < kMinTrackLength)
    {
        std::cout << "# WARNING: At least " << kMinTrackLength << " poses are needed. Aborting..." << std::endl;

        return;
    }

    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > odoMotions;
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > camMotions;

    for (size_t i = 1; i < odoPoses.size(); ++i)
    {
        Eigen::Matrix4d relativeOdometerPose = odoPoses.at(i)->pose().inverse() * odoPoses.at(i - 1)->pose();
        odoMotions.push_back(relativeOdometerPose);

        Eigen::Matrix4d relativeCameraPose = camPoses.at(i) * camPoses.at(i - 1).inverse();
        camMotions.push_back(relativeCameraPose);
    }

    if (!mOdoCamCalib.addMotionSegment(camMotions, odoMotions))
    {
        std::cout << "# ERROR: Numbers of camera and odometry motions do not match." << std::endl;
        exit(0);
    }

    mFrameSegments.push_back(frameSegment);
}

CamRigThread::CamRigThread(const std::vector<CataCameraCalibration*>& cameraCalib,
                           CameraRigExtrinsics& cameraRigExt,
                           SparseGraph& graph,
                           int beginStage,
                           bool findLoopClosures,
                           bool saveWorkingData,
                           std::string dataDir,
                           bool verbose)
 : mThread(0)
 , mRunning(false)
 , mCameraCalib(cameraCalib)
 , mCameraRigExt(cameraRigExt)
 , mGraph(graph)
 , mBeginStage(beginStage)
 , mFindLoopClosures(findLoopClosures)
 , mSaveWorkingData(saveWorkingData)
 , mDataDir(dataDir)
 , mVerbose(verbose)
{

}

CamRigThread::~CamRigThread()
{
    g_return_if_fail(mThread == 0);
}

void
CamRigThread::launch(void)
{
    mThread = Glib::Threads::Thread::create(sigc::mem_fun(*this, &CamRigThread::threadFunction));
}

void
CamRigThread::join(void)
{
    if (mRunning)
    {
        mThread->join();
    }
    mThread = 0;
}

bool
CamRigThread::running(void) const
{
    return mRunning;
}

sigc::signal<void>&
CamRigThread::signalFinished(void)
{
    return mSignalFinished;
}

void
CamRigThread::threadFunction(void)
{
    mRunning = true;

    std::vector<CataCameraParameters> cameraParameters;
    for (size_t i = 0; i < mCameraCalib.size(); ++i)
    {
        cameraParameters.push_back(mCameraCalib.at(i)->cameraParameters());
    }

    CameraRigBA ba(cameraParameters, mGraph, mCameraRigExt);
    ba.setVerbose(mVerbose);
    ba.run(mBeginStage, mFindLoopClosures, mSaveWorkingData, mDataDir);

    mRunning = false;

    mSignalFinished();
}

CameraRigCalibration::CameraRigCalibration(std::vector<AtomicData<cv::Mat>* >& images,
                                           std::vector<CataCameraCalibration*>& cameraCalib,
                                           SensorDataBuffer<OdometerPtr>& odometerBuffer,
                                           SensorDataBuffer<PosePtr>& gpsInsBuffer,
                                           const Options& options)
 : mMainLoop(Glib::MainLoop::create())
 , mOdoCamThreads(CAMERA_COUNT)
 , mImages(images)
 , mCameraCalib(cameraCalib)
 , mOdometerBuffer(odometerBuffer)
 , mGpsInsBuffer(gpsInsBuffer)
 , mExtrinsics(CAMERA_COUNT)
 , mStatuses(CAMERA_COUNT)
 , mSketches(CAMERA_COUNT)
 , mOptions(options)
{
    for (size_t i = 0; i < mOdoCamThreads.size(); ++i)
    {
        OdoCamThread* thread = new OdoCamThread(options.nMotions, i, mImages.at(i), mCameraCalib.at(i),
                                                mOdometerBuffer, mInterpOdometerBuffer, mOdometerBufferMutex,
                                                mGpsInsBuffer, mInterpGpsInsBuffer, mGpsInsBufferMutex,
                                                mStatuses.at(i), mSketches.at(i), mStop,
                                                options.saveImages, options.verbose);
        mOdoCamThreads.at(i) = thread;
        thread->signalFinished().connect(sigc::bind(sigc::mem_fun(*this, &CameraRigCalibration::onOdoCamThreadFinished), thread));
    }

    mCamRigThread = new CamRigThread(mCameraCalib, mExtrinsics, mGraph, options.beginStage, options.findLoopClosures, options.saveWorkingData, options.dataDir, options.verbose);
    mCamRigThread->signalFinished().connect(sigc::bind(sigc::mem_fun(*this, &CameraRigCalibration::onCamRigThreadFinished), mCamRigThread));

    for (size_t i = 0; i < mSketches.size(); ++i)
    {
        mSketches.at(i) = cv::Mat(1280, 800, CV_8UC3);
        mSketches.at(i) = cv::Scalar(0);
    }
}

CameraRigCalibration::~CameraRigCalibration()
{

}

void
CameraRigCalibration::run(void)
{
    if (mOptions.beginStage == 0)
    {
        CalibrationWindow::instance()->setKeyboardHandler(&CameraRigCalibration::keyboardHandler);

        Glib::signal_timeout().connect(sigc::mem_fun(*this, &CameraRigCalibration::displayHandler), 100);

        std::cout << "# INFO: Running odometer-camera calibration for each of the " << CAMERA_COUNT << " cameras." << std::endl;

        // run odometer-camera calibration for each camera
        Glib::signal_idle().connect_once(sigc::mem_fun(*this, &CameraRigCalibration::launchOdoCamThreads));
        mMainLoop->run();

        std::cout << "# INFO: Completed odometer-camera calibration for all cameras." << std::endl;

        if (mOptions.saveWorkingData)
        {
            if (!boost::filesystem::exists(mOptions.dataDir))
            {
                boost::filesystem::create_directory(mOptions.dataDir);
            }

            // save intermediate data
            std::cout << "# INFO: Saving intermediate data... " << std::flush;

            double tsStart = timeInSeconds();

            boost::filesystem::path extrinsicPath(mOptions.dataDir);
            extrinsicPath /= "tmp_extrinsic_0.txt";

            boost::filesystem::path graphPath(mOptions.dataDir);
            graphPath /= "frames_0.sg";

            mExtrinsics.writeToFile(extrinsicPath.string());
            mGraph.writeToBinaryFile(graphPath.string());

            std::cout << "Done. Took " << std::fixed << std::setprecision(2) << timeInSeconds() - tsStart << "s." << std::endl;
        }
    }
    else
    {
        std::cout << "# INFO: Reading intermediate data... " << std::flush;

        std::ostringstream oss;
        oss << "tmp_extrinsic_" << mOptions.beginStage - 1 << ".txt";

        boost::filesystem::path extrinsicPath(mOptions.dataDir);
        extrinsicPath /= oss.str();

        oss.str(""); oss.clear();
        oss << "frames_" << mOptions.beginStage - 1 << ".sg";

        boost::filesystem::path graphPath(mOptions.dataDir);
        graphPath /= oss.str();

        double tsStart = timeInSeconds();

        if (!mExtrinsics.readFromFile(extrinsicPath.string()))
        {
            std::cout << "# ERROR: Working data in file " << extrinsicPath.string() << " is missing." << std::endl;
            exit(1);
        }

        if (!mGraph.readFromBinaryFile(graphPath.string()))
        {
            std::cout << "# ERROR: Working data in file " << graphPath.string() << " is missing." << std::endl;
            exit(1);
        }

        std::cout << "Done. Took " << std::fixed << std::setprecision(2) << timeInSeconds() - tsStart << "s." << std::endl;
    }

    std::cout << "# INFO: Running camera rig calibration." << std::endl;

    double tsStart = timeInSeconds();

    // run sliding window BA with fixed poses
    mCamRigThread->launch();
    mMainLoop->run();

    std::cout << "# INFO: Camera rig calibration took " << timeInSeconds() - tsStart << "s." << std::endl;

    std::cout << "# INFO: Completed camera rig calibration." << std::endl;
}

const CameraRigExtrinsics&
CameraRigCalibration::extrinsics(void) const
{
    return mExtrinsics;
}

void
CameraRigCalibration::launchOdoCamThreads(void)
{
    std::for_each(mOdoCamThreads.begin(), mOdoCamThreads.end(), std::mem_fun(&OdoCamThread::launch));
}

void
CameraRigCalibration::onOdoCamThreadFinished(OdoCamThread* odoCamThread)
{
    odoCamThread->join();
    mExtrinsics.setGlobalCameraPose(odoCamThread->cameraIdx(), odoCamThread->camOdoTransform());
    mGraph.frameSegments(odoCamThread->cameraIdx()) = odoCamThread->frameSegments();

    if (mOptions.verbose)
    {
        double minError, maxError, avgError;
        odoCamThread->reprojectionError(minError, maxError, avgError);

        std::cout << "# INFO: Reprojection error for camera " << odoCamThread->cameraIdx()
                  << ": avg = " << avgError
                  << " px | max = " << maxError << " px" << std::endl;
    }

    if (std::find_if(mOdoCamThreads.begin(), mOdoCamThreads.end(), std::mem_fun(&OdoCamThread::running)) == mOdoCamThreads.end())
    {
        mMainLoop->quit();
    }
}

void
CameraRigCalibration::onCamRigThreadFinished(CamRigThread* camRigThread)
{
    camRigThread->join();

    mMainLoop->quit();
}

bool
CameraRigCalibration::displayHandler(void)
{
    CalibrationWindow::instance()->dataMutex().lock();

    CalibrationWindow::instance()->frontText().assign(mStatuses.at(0));
    CalibrationWindow::instance()->leftText().assign(mStatuses.at(1));
    CalibrationWindow::instance()->rearText().assign(mStatuses.at(2));
    CalibrationWindow::instance()->rightText().assign(mStatuses.at(3));

    mSketches.at(0).copyTo(CalibrationWindow::instance()->frontView());
    mSketches.at(1).copyTo(CalibrationWindow::instance()->leftView());
    mSketches.at(2).copyTo(CalibrationWindow::instance()->rearView());
    mSketches.at(3).copyTo(CalibrationWindow::instance()->rightView());

    CalibrationWindow::instance()->dataMutex().unlock();

    return true;
}

void
CameraRigCalibration::keyboardHandler(unsigned char key, int x, int y)
{
    switch(key)
    {
    case 's':
        mStop = true;
        break;

    default:
        break;
    }
}

}
