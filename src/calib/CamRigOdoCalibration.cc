#include "camodocal/calib/CamRigOdoCalibration.h"

#include <iomanip>
#include <iostream>

#include "../gpl/EigenUtils.h"
#include "CamOdoThread.h"
#include "CamOdoWatchdogThread.h"
#include "CamRigThread.h"
#ifdef VCHARGE_VIZ
#include "../../../../library/gpl/CameraEnums.h"
#include "../../../../visualization/overlay/GLOverlayExtended.h"
#include "CalibrationWindow.h"
#endif

namespace camodocal
{

bool CamRigOdoCalibration::mStop = false;

CamRigOdoCalibration::CamRigOdoCalibration(std::vector<CameraPtr>& cameras,
                                           const Options& options)
 : mMainLoop(Glib::MainLoop::create())
 , mCamOdoThreads(cameras.size())
 , mImages(cameras.size())
 , mCameras(cameras)
 , mOdometryBuffer(1000)
 , mGpsInsBuffer(1000)
 , mExtrinsics(cameras.size())
 , mStatuses(cameras.size())
 , mSketches(cameras.size())
 , mCamOdoCompleted(boost::extents[cameras.size()])
 , mOptions(options)
 , mRunning(false)
{
    for (size_t i = 0; i < mCamOdoThreads.size(); ++i)
    {
        mImages.at(i) = new AtomicData<cv::Mat>();
        mCamOdoCompleted[i] = false;

        CamOdoThread* thread = new CamOdoThread(options.poseSource, options.nMotions, i, options.preprocessImages,
                                                mImages.at(i), mCameras.at(i),
                                                mOdometryBuffer, mInterpOdometryBuffer, mOdometryBufferMutex,
                                                mGpsInsBuffer, mInterpGpsInsBuffer, mGpsInsBufferMutex,
                                                mStatuses.at(i), mSketches.at(i), mCamOdoCompleted[i], mStop,
                                                options.saveImages, options.verbose);
        mCamOdoThreads.at(i) = thread;
        thread->signalFinished().connect(sigc::bind(sigc::mem_fun(*this, &CamRigOdoCalibration::onCamOdoThreadFinished), thread));
    }

    mCamOdoWatchdogThread = new CamOdoWatchdogThread(mCamOdoCompleted, mStop);

    mCamRigThread = new CamRigThread(mCameras, mExtrinsics, mGraph, options.beginStage, options.findLoopClosures, options.optimizeIntrinsics, options.saveWorkingData, options.dataDir, options.verbose);
    mCamRigThread->signalFinished().connect(sigc::bind(sigc::mem_fun(*this, &CamRigOdoCalibration::onCamRigThreadFinished), mCamRigThread));

    for (size_t i = 0; i < mSketches.size(); ++i)
    {
        mSketches.at(i) = cv::Mat(cameras.at(i)->imageHeight(), cameras.at(i)->imageWidth(), CV_8UC3);
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

    frame->notifyData();

    if (mOptions.mode == OFFLINE)
    {
        frame->waitForProcessingDone();
    }
}

void
CamRigOdoCalibration::addOdometry(double x, double y, double yaw,
                                  uint64_t timestamp)
{
    OdometryPtr odometry(new Odometry);
    odometry->x() = x;
    odometry->y() = y;
    odometry->yaw() = yaw;
    odometry->timeStamp() = timestamp;

    mOdometryBuffer.push(timestamp, odometry);
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
CamRigOdoCalibration::start(void)
{
    if (mOptions.beginStage == 0)
    {
#ifdef VCHARGE_VIZ
        CalibrationWindow::instance()->setKeyboardHandler(&CamRigOdoCalibration::keyboardHandler);

        Glib::signal_timeout().connect(sigc::mem_fun(*this, &CamRigOdoCalibration::displayHandler), 100);
#endif

        std::cout << "# INFO: Running odometry-camera calibration for each of the " << mCameras.size() << " cameras." << std::endl;

        // run odometry-camera calibration for each camera
        Glib::signal_idle().connect_once(sigc::mem_fun(*this, &CamRigOdoCalibration::launchCamOdoThreads));
        mMainLoop->run();

        mRunning = true;

        std::cout << "# INFO: Completed odometry-camera calibration for all cameras." << std::endl;

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

    mRunning = false;
}

void
CamRigOdoCalibration::run(void)
{
    mStop = true;
}

bool
CamRigOdoCalibration::isRunning(void) const
{
    return mRunning;
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

    mCamOdoWatchdogThread->launch();
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

bool
CamRigOdoCalibration::displayHandler(void)
{
#ifdef VCHARGE_VIZ
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
#endif

    return true;
}

void
CamRigOdoCalibration::keyboardHandler(unsigned char key, int x, int y)
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
