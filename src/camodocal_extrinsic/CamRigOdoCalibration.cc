#include "camodocal/CamRigOdoCalibration.h"

#include <iomanip>
#include <iostream>

#include "../gpl/EigenUtils.h"
#include "../visual_odometry/FeatureTracker.h"
#include "../visual_odometry/ReprojectionError.h"

#include "CameraRigBA.h"
#include "utils.h"

namespace camodocal
{

bool CamRigOdoCalibration::mStop = false;

CamOdoThread::CamOdoThread(int nMotions, int cameraIdx,
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
    mCamOdoCalib.setVerbose(verbose);
    mCamOdoCalib.setMotionCount(nMotions);
}

CamOdoThread::~CamOdoThread()
{
    g_return_if_fail(mThread == 0);
}

int
CamOdoThread::cameraIdx(void) const
{
    return mCameraIdx;
}

const Eigen::Matrix4d&
CamOdoThread::camOdoTransform(void) const
{
    return mCamOdoTransform;
}

const std::vector<FrameSegment>&
CamOdoThread::frameSegments(void) const
{
    return mFrameSegments;
}

void
CamOdoThread::reprojectionError(double& minError, double& maxError, double& avgError) const
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
CamOdoThread::launch(void)
{
    mThread = Glib::Threads::Thread::create(sigc::mem_fun(*this, &CamOdoThread::threadFunction));
}

void
CamOdoThread::join(void)
{
    if (mRunning)
    {
        mThread->join();
    }
    mThread = 0;
}

bool
CamOdoThread::running(void) const
{
    return mRunning;
}

sigc::signal<void>&
CamOdoThread::signalFinished(void)
{
    return mSignalFinished;
}

void
CamOdoThread::threadFunction(void)
{
    mRunning = true;

    TemporalFeatureTracker tracker(mCameraCalib->cameraParameters(),
                                   SURF_DETECTOR, SURF_DESCRIPTOR, RATIO, false);
    tracker.setVerbose(mCamOdoCalib.getVerbose());

    uint64_t prevTimeStamp = 0;
    Eigen::Vector2d prevPos(0.0, 0.0);
    bool receivedOdometer = false;

    cv::Mat image;
    cv::Mat colorImage;

    int trackBreaks = 0;

    std::vector<OdometerPtr> odometerPoses;

    bool halt = false;

    while (!halt && !mCamOdoCalib.motionsEnough())
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
            bool camValid = tracker.addFrame(frame, cv::Mat(), R, t);

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
                 mCamOdoCalib.getCurrentMotionCount() + odometerPoses.size() > mCamOdoCalib.getMotionCount()))
            {
                addCamOdoCalibData(tracker.getPoses(), odometerPoses, tracker.getFrames());

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

            prevTimeStamp = timeStamp;
        }

        int currentMotionCount = 0;
        if (odometerPoses.size() >= kMinTrackLength)
        {
            currentMotionCount = odometerPoses.size() - 1;
        }

        if (!tracker.getSketch().empty())
        {
            tracker.getSketch().copyTo(mSketch);
        }
        else
        {
            colorImage.copyTo(mSketch);
        }
    }

    std::cout << "# INFO: Calibrating odometer - camera " << mCameraIdx << "..." << std::endl;

//    mCamOdoCalib.writeMotionSegmentsToFile(filename);

    Eigen::Matrix4d H_cam_odo;
    mCamOdoCalib.calibrate(H_cam_odo);

    std::cout << "# INFO: Finished calibrating odometer - camera " << mCameraIdx << "..." << std::endl;
    std::cout << "Rotation: " << std::endl << H_cam_odo.block<3,3>(0,0) << std::endl;
    std::cout << "Translation: " << std::endl << H_cam_odo.block<3,1>(0,3).transpose() << std::endl;

    mCamOdoTransform = H_cam_odo;

    mRunning = false;

    mSignalFinished();
}

void
CamOdoThread::addCamOdoCalibData(const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> >& camPoses,
                                 const std::vector<OdometerPtr>& odoPoses,
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

    if (!mCamOdoCalib.addMotionSegment(camMotions, odoMotions))
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

    std::vector<CataCamera::Parameters> cameraParameters;
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

CamRigOdoCalibration::CamRigOdoCalibration(std::vector<CataCameraCalibration*>& cameraCalib,
                                           const Options& options)
 : mMainLoop(Glib::MainLoop::create())
 , mCamOdoThreads(cameraCalib.size())
 , mImages(cameraCalib.size())
 , mCameraCalib(cameraCalib)
 , mOdometerBuffer(1000)
 , mGpsInsBuffer(1000)
 , mExtrinsics(cameraCalib.size())
 , mStatuses(cameraCalib.size())
 , mSketches(cameraCalib.size())
 , mOptions(options)
{
    for (size_t i = 0; i < mCamOdoThreads.size(); ++i)
    {
        CamOdoThread* thread = new CamOdoThread(options.nMotions, i, mImages.at(i), mCameraCalib.at(i),
                                                mOdometerBuffer, mInterpOdometerBuffer, mOdometerBufferMutex,
                                                mGpsInsBuffer, mInterpGpsInsBuffer, mGpsInsBufferMutex,
                                                mStatuses.at(i), mSketches.at(i), mStop,
                                                options.saveImages, options.verbose);
        mCamOdoThreads.at(i) = thread;
        thread->signalFinished().connect(sigc::bind(sigc::mem_fun(*this, &CamRigOdoCalibration::onCamOdoThreadFinished), thread));
    }

    mCamRigThread = new CamRigThread(mCameraCalib, mExtrinsics, mGraph, options.beginStage, options.findLoopClosures, options.saveWorkingData, options.dataDir, options.verbose);
    mCamRigThread->signalFinished().connect(sigc::bind(sigc::mem_fun(*this, &CamRigOdoCalibration::onCamRigThreadFinished), mCamRigThread));

    for (size_t i = 0; i < mSketches.size(); ++i)
    {
        mSketches.at(i) = cv::Mat(1280, 800, CV_8UC3);
        mSketches.at(i) = cv::Scalar(0);
    }
}

CamRigOdoCalibration::~CamRigOdoCalibration()
{

}

void
CamRigOdoCalibration::addFrame(int cameraIdx, const cv::Mat& image,
                               uint64_t timestamp)
{
    AtomicData<cv::Mat>* frame = mImages.at(cameraIdx);

    frame->lockData();

    image.copyTo(frame->data());

    frame->timeStamp() = timestamp;
    frame->available() = true;

    frame->unlockData();
}

void
CamRigOdoCalibration::addOdometry(double x, double y, double yaw,
                                  uint64_t timestamp)
{
    OdometerPtr odometer(new Odometer);
    odometer->x() = x;
    odometer->y() = y;
    odometer->yaw() = yaw;
    odometer->timeStamp() = timestamp;

    mOdometerBuffer.push(timestamp, odometer);
}

void
CamRigOdoCalibration::addGpsIns(double lat, double lon,
                                double roll, double pitch, double yaw,
                                uint64_t timestamp)
{
   // convert latitude/longitude to UTM coordinates
    double utmX, utmY;
    std::string utmZone;
    LLtoUTM(lat, lon, utmX, utmY, utmZone);

    PosePtr pose(new Pose);
    pose->rotation() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                       * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                       * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    pose->translation() = Eigen::Vector3d(utmX, utmY, 0.0);

    pose->timeStamp() = timestamp;

    mGpsInsBuffer.push(timestamp, pose);
}

void
CamRigOdoCalibration::run(void)
{
    if (mOptions.beginStage == 0)
    {
        std::cout << "# INFO: Running odometer-camera calibration for each of the " << mCameraCalib.size() << " cameras." << std::endl;

        // run odometer-camera calibration for each camera
        Glib::signal_idle().connect_once(sigc::mem_fun(*this, &CamRigOdoCalibration::launchCamOdoThreads));
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
CamRigOdoCalibration::extrinsics(void) const
{
    return mExtrinsics;
}

void
CamRigOdoCalibration::launchCamOdoThreads(void)
{
    std::for_each(mCamOdoThreads.begin(), mCamOdoThreads.end(), std::mem_fun(&CamOdoThread::launch));
}

void
CamRigOdoCalibration::onCamOdoThreadFinished(CamOdoThread* odoCamThread)
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

    if (std::find_if(mCamOdoThreads.begin(), mCamOdoThreads.end(), std::mem_fun(&CamOdoThread::running)) == mCamOdoThreads.end())
    {
        mMainLoop->quit();
    }
}

void
CamRigOdoCalibration::onCamRigThreadFinished(CamRigThread* camRigThread)
{
    camRigThread->join();

    mMainLoop->quit();
}

}
