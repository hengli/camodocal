#include "camodocal/calib/CamRigOdoCalibration.h"

#include <boost/filesystem.hpp>
#include <boost/icl/interval_map.hpp>
#include <boost/thread.hpp>
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

bool CamRigOdoCalibration::m_stop = false;

CamRigOdoCalibration::CamRigOdoCalibration(std::vector<CameraPtr>& cameras,
                                           const Options& options)
 : m_mainLoop(Glib::MainLoop::create())
 , m_camOdoThreads(cameras.size())
 , m_images(cameras.size())
 , m_cameras(cameras)
 , m_odometryBuffer(1000)
 , m_gpsInsBuffer(1000)
 , m_extrinsics(cameras.size())
 , m_statuses(cameras.size())
 , m_sketches(cameras.size())
 , m_camOdoCompleted(boost::extents[cameras.size()])
 , m_options(options)
 , m_running(false)
{
    for (size_t i = 0; i < m_camOdoThreads.size(); ++i)
    {
        m_images.at(i) = new AtomicData<cv::Mat>();
        m_camOdoCompleted[i] = false;

        CamOdoThread* thread = new CamOdoThread(options.poseSource, options.nMotions, i, options.preprocessImages,
                                                m_images.at(i), m_cameras.at(i),
                                                m_odometryBuffer, m_interpOdometryBuffer, m_odometryBufferMutex,
                                                m_gpsInsBuffer, m_interpGpsInsBuffer, m_gpsInsBufferMutex,
                                                m_statuses.at(i), m_sketches.at(i), m_camOdoCompleted[i], m_stop,
                                                options.saveImages, options.verbose);
        m_camOdoThreads.at(i) = thread;
        thread->signalFinished().connect(sigc::bind(sigc::mem_fun(*this, &CamRigOdoCalibration::onCamOdoThreadFinished), thread));
    }

    m_camOdoWatchdogThread = new CamOdoWatchdogThread(m_camOdoCompleted, m_stop);

    m_camRigThread = new CamRigThread(m_cameras, m_extrinsics, m_graph, options.beginStage, options.findLoopClosures, options.optimizeIntrinsics, options.saveWorkingData, options.dataDir, options.verbose);
    m_camRigThread->signalFinished().connect(sigc::bind(sigc::mem_fun(*this, &CamRigOdoCalibration::onCamRigThreadFinished), m_camRigThread));

    for (size_t i = 0; i < m_sketches.size(); ++i)
    {
        m_sketches.at(i) = cv::Mat(cameras.at(i)->imageHeight(), cameras.at(i)->imageWidth(), CV_8UC3);
        m_sketches.at(i) = cv::Scalar(0);
    }
}

CamRigOdoCalibration::~CamRigOdoCalibration()
{

}

void
CamRigOdoCalibration::addFrame(int cameraId, const cv::Mat& image,
                               uint64_t timestamp)
{
    AtomicData<cv::Mat>* frame = m_images.at(cameraId);

    frame->lockData();

    image.copyTo(frame->data());

    frame->timeStamp() = timestamp;

    frame->unlockData();

    if (m_options.mode == OFFLINE)
    {
        frame->waitForProcessingDone();
    }
}

void
CamRigOdoCalibration::addFrameSet(const std::vector<cv::Mat>& images,
                                  uint64_t timestamp)
{
    if (images.size() != m_cameras.size())
    {
        std::cout << "# WARNING: Number of images does not match number of cameras." << std::endl;
        return;
    }

    std::vector<boost::shared_ptr<boost::thread> > threads(m_cameras.size());
    for (size_t i = 0; i < m_cameras.size(); ++i)
    {
        threads.at(i).reset(new boost::thread(boost::bind(&CamRigOdoCalibration::addFrame, this,
                                                          i, images.at(i), timestamp)));
    }

    for (size_t i = 0; i < m_cameras.size(); ++i)
    {
        threads.at(i)->join();
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

    m_odometryBuffer.push(timestamp, odometry);
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

    m_gpsInsBuffer.push(timestamp, pose);
}

void
CamRigOdoCalibration::start(void)
{
    if (m_options.beginStage == 0)
    {
#ifdef VCHARGE_VIZ
        CalibrationWindow::instance()->setKeyboardHandler(&CamRigOdoCalibration::keyboardHandler);

        Glib::signal_timeout().connect(sigc::mem_fun(*this, &CamRigOdoCalibration::displayHandler), 100);
#endif

        std::cout << "# INFO: Running camera-odometry calibration for each of the " << m_cameras.size() << " cameras." << std::endl;

        // run odometry-camera calibration for each camera
        Glib::signal_idle().connect_once(sigc::mem_fun(*this, &CamRigOdoCalibration::launchCamOdoThreads));
        m_mainLoop->run();

        buildGraph();

        m_running = true;

        std::cout << "# INFO: Completed camera-odometry calibration for all cameras." << std::endl;

        if (m_options.saveWorkingData)
        {
            if (!boost::filesystem::exists(m_options.dataDir))
            {
                boost::filesystem::create_directory(m_options.dataDir);
            }

            // save intermediate data
            std::cout << "# INFO: Saving intermediate data... " << std::flush;

            double tsStart = timeInSeconds();

            boost::filesystem::path extrinsicPath(m_options.dataDir);
            extrinsicPath /= "tmp_extrinsic_0.txt";

            boost::filesystem::path graphPath(m_options.dataDir);
            graphPath /= "frames_0.sg";

            m_extrinsics.writeToFile(extrinsicPath.string());
            m_graph.writeToBinaryFile(graphPath.string());

            std::cout << "Done. Took " << std::fixed << std::setprecision(2) << timeInSeconds() - tsStart << "s." << std::endl;
        }
    }
    else
    {
        std::cout << "# INFO: Reading intermediate data... " << std::flush;

        std::ostringstream oss;
        oss << "tmp_extrinsic_" << m_options.beginStage - 1 << ".txt";

        boost::filesystem::path extrinsicPath(m_options.dataDir);
        extrinsicPath /= oss.str();

        oss.str(""); oss.clear();
        oss << "frames_" << m_options.beginStage - 1 << ".sg";

        boost::filesystem::path graphPath(m_options.dataDir);
        graphPath /= oss.str();

        double tsStart = timeInSeconds();

        if (!m_extrinsics.readFromFile(extrinsicPath.string()))
        {
            std::cout << "# ERROR: Working data in file " << extrinsicPath.string() << " is missing." << std::endl;
            exit(1);
        }

        if (!m_graph.readFromBinaryFile(graphPath.string()))
        {
            std::cout << "# ERROR: Working data in file " << graphPath.string() << " is missing." << std::endl;
            exit(1);
        }

        std::cout << "Done. Took " << std::fixed << std::setprecision(2) << timeInSeconds() - tsStart << "s." << std::endl;
    }

    std::cout << "# INFO: Running camera rig calibration." << std::endl;

    double tsStart = timeInSeconds();

    // run calibration steps
    m_camRigThread->launch();
    m_mainLoop->run();

    std::cout << "# INFO: Camera rig calibration took " << timeInSeconds() - tsStart << "s." << std::endl;

    std::cout << "# INFO: Completed camera rig calibration." << std::endl;

    m_running = false;
}

void
CamRigOdoCalibration::run(void)
{
    m_stop = true;
}

bool
CamRigOdoCalibration::isRunning(void) const
{
    return m_running;
}

const CameraRigExtrinsics&
CamRigOdoCalibration::extrinsics(void) const
{
    return m_extrinsics;
}

void
CamRigOdoCalibration::launchCamOdoThreads(void)
{
    std::for_each(m_camOdoThreads.begin(), m_camOdoThreads.end(), std::mem_fun(&CamOdoThread::launch));

    m_camOdoWatchdogThread->launch();
}

void
CamRigOdoCalibration::onCamOdoThreadFinished(CamOdoThread* camOdoThread)
{
    camOdoThread->join();

    if (m_options.verbose)
    {
        double minError, maxError, avgError;
        camOdoThread->reprojectionError(minError, maxError, avgError);

        std::cout << "# INFO: Reprojection error for camera " << camOdoThread->cameraId()
                  << ": avg = " << avgError
                  << " px | max = " << maxError << " px" << std::endl;
    }

    if (std::find_if(m_camOdoThreads.begin(), m_camOdoThreads.end(), std::mem_fun(&CamOdoThread::running)) == m_camOdoThreads.end())
    {
        m_mainLoop->quit();
    }
}

void
CamRigOdoCalibration::onCamRigThreadFinished(CamRigThread* camRigThread)
{
    camRigThread->join();

    m_mainLoop->quit();
}

bool compareFrameTimeStamp(FramePtr f1, FramePtr f2)
{
    return (f1->cameraPose()->timeStamp() < f2->cameraPose()->timeStamp());
}

void
CamRigOdoCalibration::buildGraph(void)
{
    boost::icl::interval_map<uint64_t, std::set<int> > intervals;
    std::set<int> cameraIdSets[m_camOdoThreads.size()];

    for (size_t i = 0; i < m_camOdoThreads.size(); ++i)
    {
        CamOdoThread* camOdoThread = m_camOdoThreads.at(i);

        m_extrinsics.setGlobalCameraPose(camOdoThread->cameraId(),
                                         camOdoThread->camOdoTransform());

        cameraIdSets[i].insert(camOdoThread->cameraId());

        for (size_t j = 0; j < camOdoThread->frameSegments().size(); ++j)
        {
            uint64_t start = camOdoThread->frameSegments().at(j).front()->cameraPose()->timeStamp();
            uint64_t end = camOdoThread->frameSegments().at(j).back()->cameraPose()->timeStamp();

            intervals += std::make_pair(boost::icl::interval<uint64_t>::right_open(start, end),
                                        cameraIdSets[i]);
        }
    }

    uint64_t lastIntervalEnd = 0;
    size_t intervalCounter = 0;

    boost::icl::interval_map<uint64_t, std::set<int> >::iterator it = intervals.begin();
    while (it != intervals.end())
    {
        boost::icl::interval<uint64_t>::type interval = it->first;
        std::set<int> cameraIds = it->second;

        uint64_t start = interval.lower();
        uint64_t end = interval.upper();

        if (start != lastIntervalEnd)
        {
            m_graph.frameSetSegments().resize(m_graph.frameSetSegments().size() + 1);
        }

        std::vector<FramePtr> frames;

        std::set<int>::iterator itCameraId = cameraIds.begin();
        while (itCameraId != cameraIds.end())
        {
            int cameraId = *itCameraId;

            CamOdoThread* camOdoThread = m_camOdoThreads.at(cameraId);

            for (size_t j = 0; j < camOdoThread->frameSegments().size(); ++j)
            {
                const std::vector<FramePtr>& frameSegment = camOdoThread->frameSegments().at(j);

                for (size_t k = 0; k < frameSegment.size(); ++k)
                {
                    const FramePtr& frame = frameSegment.at(k);

                    uint64_t timestamp = frame->cameraPose()->timeStamp();
                    if (timestamp < start || timestamp > end)
                    {
                        continue;
                    }

                    frames.push_back(frame);
                }
            }

            ++itCameraId;
        }

        // sort frames by timestamp
        std::sort(frames.begin(), frames.end(), compareFrameTimeStamp);

        size_t frameId = 0;
        while (frameId < frames.size())
        {
            FrameSetPtr frameSet(new FrameSet);
            frameSet->frames().resize(m_cameras.size());
            frameSet->systemPose() = frames.at(frameId)->systemPose();
            frameSet->odometryMeasurement() = frames.at(frameId)->odometryMeasurement();
            frameSet->gpsInsMeasurement() = frames.at(frameId)->gpsInsMeasurement();

            uint64_t timestamp = frames.at(frameId)->cameraPose()->timeStamp();
            while (frameId < frames.size() &&
                   frames.at(frameId)->cameraPose()->timeStamp() == timestamp)
            {
                frameSet->frames().at(frames.at(frameId)->cameraId()) = frames.at(frameId);

                ++frameId;
            }

            m_graph.frameSetSegments().back().push_back(frameSet);
        }

        lastIntervalEnd = end;

        ++intervalCounter;
        ++it;
    }
}

bool
CamRigOdoCalibration::displayHandler(void)
{
#ifdef VCHARGE_VIZ
    CalibrationWindow::instance()->dataMutex().lock();

    CalibrationWindow::instance()->frontText().assign(m_statuses.at(0));
    CalibrationWindow::instance()->leftText().assign(m_statuses.at(1));
    CalibrationWindow::instance()->rearText().assign(m_statuses.at(2));
    CalibrationWindow::instance()->rightText().assign(m_statuses.at(3));

    m_sketches.at(0).copyTo(CalibrationWindow::instance()->frontView());
    m_sketches.at(1).copyTo(CalibrationWindow::instance()->leftView());
    m_sketches.at(2).copyTo(CalibrationWindow::instance()->rearView());
    m_sketches.at(3).copyTo(CalibrationWindow::instance()->rightView());

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
        m_stop = true;
        break;

    default:
        break;
    }
}

}
