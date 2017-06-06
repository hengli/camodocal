#include "camodocal/calib/CamRigOdoCalibration.h"

#include <boost/filesystem.hpp>
#include <boost/icl/interval_map.hpp>
#include <boost/make_shared.hpp>
#include <boost/thread.hpp>
#include <iomanip>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>

#include "../../gpl/gpl.h"
#include "camodocal/EigenUtils.h"
#include "CamOdoThread.h"
#include "CamOdoWatchdogThread.h"
#include "CameraRigBA.h"
#ifdef VCHARGE_VIZ
#include <boost/date_time/posix_time/posix_time.hpp>

#include "../../../../visualization/overlay/GLOverlayExtended.h"
#endif

namespace camodocal
{

bool CamRigOdoCalibration::m_stop = false;

CamRigOdoCalibration::CamRigOdoCalibration(std::vector<CameraPtr>& cameras,
                                           const Options& options)
 : m_camOdoThreads(cameras.size())
 , m_cameraSystem(cameras.size())
 , m_images(cameras.size())
 , m_cameras(cameras)
 , m_odometryBuffer(1000)
 , m_gpsInsBuffer(1000)
 , m_camOdoCompleted(boost::extents[cameras.size()])
 , m_sketches(cameras.size())
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
                                                m_sketches.at(i), m_camOdoCompleted[i], m_stop,
                                                options.minKeyframeDistance, options.minVOSegmentSize,
                                                options.verbose);
        m_camOdoThreads.at(i) = thread;
        thread->signalFinished().connect(boost::bind(&CamRigOdoCalibration::onCamOdoThreadFinished, this, thread));
    }

    m_camOdoWatchdogThread = new CamOdoWatchdogThread(m_camOdoCompleted, m_stop);

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
CamRigOdoCalibration::setInitialCameraOdoTransformEstimates(unsigned camIdx, const Eigen::Matrix4d& odoT)
{
    if (camIdx >= m_camOdoThreads.size()) return;
    if (!m_camOdoThreads[camIdx] || m_camOdoThreads[camIdx]->running()) return;

    m_camOdoThreads[camIdx]->setCamOdoTransformEstimate(odoT);
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
        threads.at(i) = boost::make_shared<boost::thread>(boost::bind(&CamRigOdoCalibration::addFrame, this,
                                                          i, images.at(i), timestamp));
    }

    for (size_t i = 0; i < m_cameras.size(); ++i)
    {
        threads.at(i)->join();
    }
}

void
CamRigOdoCalibration::addOdometry(double x, double y, double z,
                                  double yaw,
                                  uint64_t timestamp)
{
    OdometryPtr odometry = boost::make_shared<Odometry>();
    odometry->x() = x;
    odometry->y() = y;
    odometry->z() = z;
    odometry->yaw() = yaw;
    odometry->timeStamp() = timestamp;

    m_odometryBuffer.push(timestamp, odometry);
}

void
CamRigOdoCalibration::addGpsIns(double lat, double lon, double alt,
                                double qx, double qy, double qz, double qw,
                                uint64_t timestamp)
{
    // convert latitude/longitude to UTM coordinates
     double utmX, utmY;
     std::string utmZone;
     LLtoUTM(lat, lon, utmX, utmY, utmZone);

     PosePtr pose = boost::make_shared<Pose>();
     pose->rotation() = Eigen::Quaterniond(qw,qx,qy,qz);
     pose->translation() = Eigen::Vector3d(utmX, utmY, -alt);

     pose->timeStamp() = timestamp;

     m_gpsInsBuffer.push(timestamp, pose);
}

void
CamRigOdoCalibration::addGpsIns(double lat, double lon, double alt,
                                double roll, double pitch, double yaw,
                                uint64_t timestamp)
{
    Eigen::Quaterniond q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                       * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                       * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    addGpsIns(lat, lon, alt, q.x(), q.y(), q.z(), q.w(), timestamp);
}

void
CamRigOdoCalibration::start(void)
{
    if (m_options.beginStage == 0)
    {
        boost::asio::io_service io;
        boost::asio::deadline_timer timer(io, boost::posix_time::milliseconds(100));
        bool closeWindow = false;

        for (size_t i = 0; i < m_cameras.size(); ++i)
        {
            cv::namedWindow(m_cameras.at(i)->cameraName());
        }

        timer.async_wait(boost::bind(&CamRigOdoCalibration::displayHandler, this, &timer, &closeWindow));

        boost::thread windowThread(boost::bind(&CamRigOdoCalibration::pollWindow, this, &io, &closeWindow));

        std::cout << "# INFO: Running camera-odometry calibration for each of the " << m_cameras.size() << " cameras." << std::endl;

        // run odometry-camera calibration for each camera
        for (size_t i = 0; i < m_camOdoThreads.size(); ++i)
        {
            m_camOdoThreads.at(i)->launch();
        }

        m_camOdoWatchdogThread->launch();
        m_camOdoWatchdogThread->join();

        for (size_t i = 0; i < m_camOdoThreads.size(); ++i)
        {
            m_camOdoThreads.at(i)->join();
        }

        closeWindow = true;
        windowThread.join();

        for (size_t i = 0; i < m_cameras.size(); ++i)
        {
            cv::destroyWindow(m_cameras.at(i)->cameraName());
        }

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
            extrinsicPath /= "extrinsic_0";

            m_cameraSystem.writeToDirectory(extrinsicPath.string());

            boost::filesystem::path graphPath(m_options.dataDir);
            graphPath /= "frames_0.sg";
            m_graph.writeToBinaryFile(graphPath.string());

            std::cout << "Done. Took " << std::fixed << std::setprecision(2) << timeInSeconds() - tsStart << "s." << std::endl;
        }
    }
    else
    {
        std::cout << "# INFO: Reading intermediate data... " << std::flush;

        std::ostringstream oss;
        oss << "extrinsic_" << m_options.beginStage - 1;

        boost::filesystem::path extrinsicPath(m_options.dataDir);
        extrinsicPath /= oss.str();

        oss.str(""); oss.clear();
        oss << "frames_" << m_options.beginStage - 1 << ".sg";

        boost::filesystem::path graphPath(m_options.dataDir);
        graphPath /= oss.str();

        double tsStart = timeInSeconds();

        if (!m_cameraSystem.readFromDirectory(extrinsicPath.string()))
        {
            std::cout << "# ERROR: Working data in directory " << extrinsicPath.string() << " is missing." << std::endl;
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
    CameraRigBA ba(m_cameraSystem, m_graph, m_options.windowDistance);
    ba.setVerbose(m_options.verbose);
    ba.run(m_options.beginStage, m_options.optimizeIntrinsics, m_options.saveWorkingData, m_options.dataDir);

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

const CameraSystem&
CamRigOdoCalibration::cameraSystem(void) const
{
    return m_cameraSystem;
}

void
CamRigOdoCalibration::onCamOdoThreadFinished(CamOdoThread* camOdoThread)
{
    if (m_options.verbose)
    {
        double minError, maxError, avgError;
        camOdoThread->reprojectionError(minError, maxError, avgError);

        std::cout << "# INFO: Reprojection error for camera " << camOdoThread->cameraId()
                  << ": avg = " << avgError
                  << " px | max = " << maxError << " px" << std::endl;
    }
}

bool compareFrameTimeStamp(FramePtr f1, FramePtr f2)
{
    return (f1->cameraPose()->timeStamp() < f2->cameraPose()->timeStamp());
}

void
CamRigOdoCalibration::buildGraph(void)
{
    boost::icl::interval_map<uint64_t, std::set<int> > intervals;
    std::vector<std::set<int> > cameraIdSets(m_camOdoThreads.size());

    for (size_t i = 0; i < m_camOdoThreads.size(); ++i)
    {
        CamOdoThread* camOdoThread = m_camOdoThreads.at(i);

        m_cameraSystem.setCamera(camOdoThread->cameraId(), m_cameras.at(camOdoThread->cameraId()));
        m_cameraSystem.setGlobalCameraPose(camOdoThread->cameraId(),
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
                    if (lastIntervalEnd == 0)
                    {
                        if (timestamp < start || timestamp > end)
                        {
                            continue;
                        }
                    }
                    else
                    {
                        if (timestamp <= start || timestamp > end)
                        {
                            continue;
                        }
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
            FrameSetPtr frameSet = boost::make_shared<FrameSet>();
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

            for (size_t i = 0; i < frameSet->frames().size(); ++i)
            {
                if (frameSet->frames().at(i).get() != 0)
                {
                    frameSet->frames().at(i)->systemPose() = frameSet->systemPose();
                    frameSet->frames().at(i)->odometryMeasurement() = frameSet->odometryMeasurement();
                    frameSet->frames().at(i)->gpsInsMeasurement() = frameSet->gpsInsMeasurement();
                }
            }

            m_graph.frameSetSegments().back().push_back(frameSet);
        }

        lastIntervalEnd = end;

        ++intervalCounter;
        ++it;
    }
}

void
CamRigOdoCalibration::pollWindow(boost::asio::io_service* io, bool* stop)
{
    while (!(*stop))
    {
        io->poll();

        usleep(1000);
    }
}

void
CamRigOdoCalibration::displayHandler(boost::asio::deadline_timer* timer, bool* stop)
{
    for (size_t i = 0; i < m_cameras.size(); ++i)
    {
        cv::imshow(m_cameras.at(i)->cameraName(), m_sketches.at(i));
    }
    cv::waitKey(2);

    if (!(*stop))
    {
        timer->expires_at(timer->expires_at() + boost::posix_time::milliseconds(100));
        timer->async_wait(boost::bind(&CamRigOdoCalibration::displayHandler, this, timer, stop));
    }
}

}
