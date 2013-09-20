#include "CamOdoThread.h"

#include <iostream>

#include "../gpl/EigenUtils.h"
#include "../visual_odometry/FeatureTracker.h"
#include "utils.h"
#ifdef VCHARGE_VIZ
#include "../../../../library/gpl/CameraEnums.h"
#include "../../../../visualization/overlay/GLOverlayExtended.h"
#include "CalibrationWindow.h"
#endif

namespace camodocal
{

CamOdoThread::CamOdoThread(PoseSource poseSource, int nMotions, int cameraIdx,
                           bool preprocess,
                           AtomicData<cv::Mat>* image,
                           const CameraConstPtr& camera,
                           SensorDataBuffer<OdometryPtr>& odometryBuffer,
                           SensorDataBuffer<OdometryPtr>& interpOdometryBuffer,
                           boost::mutex& odometryBufferMutex,
                           SensorDataBuffer<PosePtr>& gpsInsBuffer,
                           SensorDataBuffer<PosePtr>& interpGpsInsBuffer,
                           boost::mutex& gpsInsBufferMutex,
                           std::string& status,
                           cv::Mat& sketch,
                           bool& completed,
                           bool& stop,
                           bool saveImages,
                           bool verbose)
 : mPoseSource(poseSource)
 , mThread(0)
 , mCameraIdx(cameraIdx)
 , mRunning(false)
 , mPreprocess(preprocess)
 , mImage(image)
 , mCamera(camera)
 , mOdometryBuffer(odometryBuffer)
 , mInterpOdometryBuffer(interpOdometryBuffer)
 , mOdometryBufferMutex(odometryBufferMutex)
 , mGpsInsBuffer(gpsInsBuffer)
 , mInterpGpsInsBuffer(interpGpsInsBuffer)
 , mGpsInsBufferMutex(gpsInsBufferMutex)
 , mStatus(status)
 , mSketch(sketch)
 , kKeyFrameDistance(0.25)
 , kMinTrackLength(15)
 , kOdometryTimeout(4.0)
 , mCompleted(completed)
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

                double error = mCamera->reprojectionError(feature3D->point(),
                                                          frame->camera()->rotation(),
                                                          frame->camera()->translation(),
                                                          Eigen::Vector2d(feature2D->keypoint().pt.x, feature2D->keypoint().pt.y));

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

    TemporalFeatureTracker tracker(mCamera,
                                   SURF_GPU_DETECTOR, SURF_GPU_DESCRIPTOR,
                                   RATIO_GPU, mPreprocess);
    tracker.setVerbose(mCamOdoCalib.getVerbose());

    FramePtr framePrev;

    cv::Mat image;
    cv::Mat colorImage;

    int trackBreaks = 0;

    std::vector<OdometryPtr> odometryPoses;

#ifdef VCHARGE_VIZ
    std::ostringstream oss;
    oss << "swba" << mCameraIdx + 1;
    vcharge::GLOverlayExtended overlay(oss.str(), VCharge::COORDINATE_FRAME_GLOBAL);
#endif

    bool halt = false;

    while (!halt)
    {
        boost::system_time timeout = boost::get_system_time() + boost::posix_time::milliseconds(10);
        while (!mImage->timedWaitForData(timeout) && !mStop)
        {
            timeout = boost::get_system_time() + boost::posix_time::milliseconds(10);
        }

        if (mStop)
        {
            std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > voPoses = tracker.getPoses();

            if (odometryPoses.size() >= kMinTrackLength)
            {
                addCamOdoCalibData(voPoses, odometryPoses, tracker.getFrames());
            }

            if (!odometryPoses.empty())
            {
                odometryPoses.erase(odometryPoses.begin(), odometryPoses.begin() + voPoses.size() - 1);
            }

            ++trackBreaks;

            halt = true;
        }
        else
        {
            mImage->lockData();
            mImage->available() = false;

            uint64_t timeStamp = mImage->timeStamp();

            if (framePrev.get() != 0 && timeStamp == framePrev->camera()->timeStamp())
            {
                mImage->unlockData();
                mImage->notifyProcessingDone();

                continue;
            }

            mImage->data().copyTo(image);

            mImage->unlockData();

            if (image.channels() == 1)
            {
                cv::cvtColor(image, colorImage, CV_GRAY2BGR);
            }
            else
            {
                image.copyTo(colorImage);
            }

            // skip if current car position is too near previous position
            OdometryPtr currOdometry;
            PosePtr currGpsIns;
            Eigen::Vector2d pos;

            if (mPoseSource == ODOMETRY && !mOdometryBuffer.current(currOdometry))
            {
                std::cout << "# WARNING: No data in odometry buffer." << std::endl;
            }
            else if (mPoseSource == GPS_INS && !mGpsInsBuffer.current(currGpsIns))
            {
                std::cout << "# WARNING: No data in GPS/INS buffer." << std::endl;
            }
            else
            {
                mOdometryBufferMutex.lock();

                OdometryPtr interpOdo;
                if (mPoseSource == ODOMETRY && !mInterpOdometryBuffer.find(timeStamp, interpOdo))
                {
                    double timeStart = timeInSeconds();
                    while (!interpolateOdometry(mOdometryBuffer, timeStamp, interpOdo))
                    {
                        if (timeInSeconds() - timeStart > kOdometryTimeout)
                        {
                            std::cout << "# ERROR: No odometry data for " << kOdometryTimeout << "s. Exiting..." << std::endl;
                            exit(1);
                        }

                        usleep(1000);
                    }

                    mInterpOdometryBuffer.push(timeStamp, interpOdo);
                }

                mOdometryBufferMutex.unlock();

                mGpsInsBufferMutex.lock();

                PosePtr interpGpsIns;
                if ((mPoseSource == GPS_INS || !mGpsInsBuffer.empty()) && !mInterpGpsInsBuffer.find(timeStamp, interpGpsIns))
                {
                    double timeStart = timeInSeconds();
                    while (!interpolatePose(mGpsInsBuffer, timeStamp, interpGpsIns))
                    {
                        if (timeInSeconds() - timeStart > kOdometryTimeout)
                        {
                            std::cout << "# ERROR: No GPS/INS data for " << kOdometryTimeout << "s. Exiting..." << std::endl;
                            exit(1);
                        }

                        usleep(1000);
                    }

                    mInterpGpsInsBuffer.push(timeStamp, interpGpsIns);
                }

                mGpsInsBufferMutex.unlock();

                Eigen::Vector3d pos;
                if (mPoseSource == ODOMETRY)
                {
                    pos = interpOdo->position();
                }
                else
                {
                    pos(0) = interpGpsIns->translation()(1);
                    pos(1) = -interpGpsIns->translation()(0);
                    pos(2) = interpGpsIns->translation()(2);
                }

                if (framePrev.get() != 0 &&
                    (pos - framePrev->odometryOpt()->position()).norm() < kKeyFrameDistance)
                {
                    mImage->notifyProcessingDone();
                    continue;
                }

                FramePtr frame(new Frame);
                image.copyTo(frame->image());

                Eigen::Matrix3d R;
                Eigen::Vector3d t;
                bool camValid = tracker.addFrame(frame, mCamera->mask(), R, t);

                // tag frame with odometry and GPS/INS data
                frame->odometryUnopt().reset(new Odometry);
                *(frame->odometryUnopt()) = *interpOdo;
                frame->odometryOpt().reset(new Odometry);
                *(frame->odometryOpt()) = *interpOdo;

                if (interpGpsIns.get() != 0)
                {
                    frame->gps_ins() = interpGpsIns;
                }

                if (mPoseSource == GPS_INS)
                {
                    OdometryPtr gpsIns(new Odometry);
                    gpsIns->timeStamp() = interpGpsIns->timeStamp();
                    gpsIns->x() = interpGpsIns->translation()(1);
                    gpsIns->y() = -interpGpsIns->translation()(0);

                    Eigen::Matrix3d R = interpGpsIns->rotation().toRotationMatrix();
                    double roll, pitch, yaw;
                    mat2RPY(R, roll, pitch, yaw);
                    gpsIns->yaw() = -yaw;

                    frame->odometryUnopt().reset(new Odometry);
                    *(frame->odometryUnopt()) = *gpsIns;
                    frame->odometryOpt().reset(new Odometry);
                    *(frame->odometryOpt()) = *gpsIns;
                }

                if (!mSaveImages)
                {
                    frame->image() = cv::Mat();
                }

                if (camValid)
                {
                    odometryPoses.push_back(frame->odometryOpt());
                }

                framePrev = frame;

                if (!camValid)
                {
                    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > voPoses = tracker.getPoses();

                    if (odometryPoses.size() >= kMinTrackLength)
                    {
                        addCamOdoCalibData(voPoses, odometryPoses, tracker.getFrames());
                    }

                    if (!odometryPoses.empty())
                    {
                        odometryPoses.erase(odometryPoses.begin(), odometryPoses.begin() + voPoses.size() - 1);
                    }

                    ++trackBreaks;
                }
            }
        }

#ifdef VCHARGE_VIZ
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
                case vcharge::CAMERA_FRONT:
                    overlay.color4f(1.0f, 0.0f, 0.0f, 0.5f);
                    break;
                case vcharge::CAMERA_LEFT:
                    overlay.color4f(0.0f, 1.0f, 0.0f, 0.5f);
                    break;
                case vcharge::CAMERA_REAR:
                    overlay.color4f(0.0f, 0.0f, 1.0f, 0.5f);
                    break;
                case vcharge::CAMERA_RIGHT:
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
#endif

        int currentMotionCount = 0;
        if (odometryPoses.size() >= kMinTrackLength)
        {
            currentMotionCount = odometryPoses.size() - 1;
        }

        std::ostringstream oss;
        oss << "# motions: " << mCamOdoCalib.getCurrentMotionCount() + currentMotionCount << " | "
            << "# track breaks: " << trackBreaks;

#ifdef VCHARGE_VIZ
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
#endif

        mImage->notifyProcessingDone();

        if (mCamOdoCalib.getCurrentMotionCount() + currentMotionCount >= mCamOdoCalib.getMotionCount())
        {
            mCompleted = true;
        }
    }

    std::cout << "# INFO: Calibrating odometry - camera " << mCameraIdx << "..." << std::endl;

//    mCamOdoCalib.writeMotionSegmentsToFile(filename);

    Eigen::Matrix4d H_cam_odo;
    mCamOdoCalib.calibrate(H_cam_odo);

    std::cout << "# INFO: Finished calibrating odometry - camera " << mCameraIdx << "..." << std::endl;
    std::cout << "Rotation: " << std::endl << H_cam_odo.block<3,3>(0,0) << std::endl;
    std::cout << "Translation: " << std::endl << H_cam_odo.block<3,1>(0,3).transpose() << std::endl;

    mCamOdoTransform = H_cam_odo;

    mRunning = false;

    mSignalFinished();
}

void
CamOdoThread::addCamOdoCalibData(const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> >& camPoses,
                                 const std::vector<OdometryPtr>& odoPoses,
                                 FrameSegment& frameSegment)
{
    if (odoPoses.size() != camPoses.size())
    {
        std::cout << "# WARNING: Numbers of odometry (" << odoPoses.size()
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
        Eigen::Matrix4d relativeOdometryPose = odoPoses.at(i)->pose().inverse() * odoPoses.at(i - 1)->pose();
        odoMotions.push_back(relativeOdometryPose);

        Eigen::Matrix4d relativeCameraPose = camPoses.at(i) * camPoses.at(i - 1).inverse();
        camMotions.push_back(relativeCameraPose);
    }

    if (!mCamOdoCalib.addMotionSegment(camMotions, odoMotions))
    {
        std::cout << "# ERROR: Numbers of odometry and camera motions do not match." << std::endl;
        exit(0);
    }

    mFrameSegments.push_back(frameSegment);
}

}
