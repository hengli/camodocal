#include "CamOdoThread.h"

#include <boost/make_shared.hpp>
#include <iostream>

#ifdef HAVE_OPENCV3
#include <opencv2/imgproc.hpp>
#endif

#include "../gpl/gpl.h"
#include "camodocal/EigenUtils.h"
#include "../visual_odometry/FeatureTracker.h"
#include "utils.h"

#ifdef VCHARGE_VIZ
#include "../../../../library/gpl/CameraEnums.h"
#endif

namespace camodocal
{

CamOdoThread::CamOdoThread(PoseSource poseSource, int nMotions, int cameraId,
                           bool preprocess,
                           AtomicData<cv::Mat>* image,
                           const CameraConstPtr& camera,
                           SensorDataBuffer<OdometryPtr>& odometryBuffer,
                           SensorDataBuffer<OdometryPtr>& interpOdometryBuffer,
                           boost::mutex& odometryBufferMutex,
                           SensorDataBuffer<PosePtr>& gpsInsBuffer,
                           SensorDataBuffer<PosePtr>& interpGpsInsBuffer,
                           boost::mutex& gpsInsBufferMutex,
                           cv::Mat& sketch,
                           bool& completed,
                           bool& stop,
                           double minKeyframeDistance,
                           size_t minVOSegmentSize,
                           bool verbose)
 : m_poseSource(poseSource)
 , m_cameraId(cameraId)
 , m_preprocess(preprocess)
 , m_running(false)
 , m_image(image)
 , m_camera(camera)
 , m_odometryBuffer(odometryBuffer)
 , m_interpOdometryBuffer(interpOdometryBuffer)
 , m_odometryBufferMutex(odometryBufferMutex)
 , m_gpsInsBuffer(gpsInsBuffer)
 , m_interpGpsInsBuffer(interpGpsInsBuffer)
 , m_gpsInsBufferMutex(gpsInsBufferMutex)
 , m_camOdoTransform(Eigen::Matrix4d::Identity())
 , m_camOdoTransformUseEstimate(false)
 , m_sketch(sketch)
 , m_completed(completed)
 , m_stop(stop)
 , k_minKeyframeDistance(minKeyframeDistance)
 , k_minVOSegmentSize(minVOSegmentSize)
 , k_odometryTimeout(4.0)
{
    m_camOdoCalib.setVerbose(verbose);
    m_camOdoCalib.setMotionCount(nMotions);
}

CamOdoThread::~CamOdoThread()
{

}

int
CamOdoThread::cameraId(void) const
{
    return m_cameraId;
}

void
CamOdoThread::setCamOdoTransformEstimate(const Eigen::Matrix4d& estimate)
{
    m_camOdoTransform = estimate;
    m_camOdoTransformUseEstimate = true;
}

void
CamOdoThread::clearCamOdoTransformEstimate()
{
    m_camOdoTransformUseEstimate = false;
}

const Eigen::Matrix4d&
CamOdoThread::camOdoTransform(void) const
{
    return m_camOdoTransform;
}

const std::vector<std::vector<FramePtr> >&
CamOdoThread::frameSegments(void) const
{
    return m_frameSegments;
}

void
CamOdoThread::reprojectionError(double& minError, double& maxError, double& avgError) const
{
    minError = std::numeric_limits<double>::max();
    maxError = std::numeric_limits<double>::min();

    size_t count = 0;
    double totalError = 0.0;

    for (size_t segmentId = 0; segmentId < m_frameSegments.size(); ++segmentId)
    {
        const std::vector<FramePtr>& segment = m_frameSegments.at(segmentId);

        for (size_t frameId = 0; frameId < segment.size(); ++frameId)
        {
            const FrameConstPtr& frame = segment.at(frameId);

            const std::vector<Point2DFeaturePtr>& features2D = frame->features2D();

            for (size_t i = 0; i < features2D.size(); ++i)
            {
                const Point2DFeatureConstPtr& feature2D = features2D.at(i);
                const Point3DFeatureConstPtr& feature3D = feature2D->feature3D();

                if (feature3D.get() == 0)
                {
                    continue;
                }

                double error = m_camera->reprojectionError(feature3D->point(),
                                                           frame->cameraPose()->rotation(),
                                                           frame->cameraPose()->translation(),
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
    m_running = true;

    m_thread = boost::make_shared<boost::thread>(&CamOdoThread::threadFunction, this);
}

void
CamOdoThread::join(void)
{
    if (m_running)
    {
        m_thread->join();
    }
}

bool
CamOdoThread::running(void) const
{
    return m_running;
}

boost::signals2::signal<void ()>&
CamOdoThread::signalFinished(void)
{
    return m_signalFinished;
}

void
CamOdoThread::threadFunction(void)
{
    TemporalFeatureTracker tracker(m_camera,
                                   SURF_GPU_DETECTOR, SURF_GPU_DESCRIPTOR,
                                   RATIO_GPU, m_preprocess, m_camOdoTransform);
    tracker.setVerbose(m_camOdoCalib.getVerbose());

    FramePtr framePrev;

    cv::Mat image;
    cv::Mat colorImage;

    int trackBreaks = 0;

    std::vector<OdometryPtr> odometryPoses;

#ifdef VCHARGE_VIZ
    std::ostringstream oss;
    oss << "swba" << m_cameraId + 1;
    vcharge::GLOverlayExtended overlay(oss.str(), VCharge::COORDINATE_FRAME_GLOBAL);
#endif

    bool halt = false;

    while (!halt)
    {
        boost::system_time timeout = boost::get_system_time() + boost::posix_time::milliseconds(10);
        while (!m_image->timedWaitForData(timeout) && !m_stop)
        {
            timeout = boost::get_system_time() + boost::posix_time::milliseconds(10);
        }

        if (m_stop)
        {
            std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > voPoses = tracker.getPoses();

            if (odometryPoses.size() >= k_minVOSegmentSize)
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
            m_image->lockData();
            m_image->available() = false;

            uint64_t timeStamp = m_image->timeStamp();

            if (framePrev.get() != 0 && timeStamp == framePrev->cameraPose()->timeStamp())
            {
                m_image->unlockData();
                m_image->notifyProcessingDone();

                continue;
            }

            m_image->data().copyTo(image);

            m_image->unlockData();

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

            if (m_poseSource == ODOMETRY && !m_odometryBuffer.current(currOdometry))
            {
                std::cout << "# WARNING: No data in odometry buffer." << std::endl;
            }
            else if (m_poseSource == GPS_INS && !m_gpsInsBuffer.current(currGpsIns))
            {
                std::cout << "# WARNING: No data in GPS/INS buffer." << std::endl;
            }
            else
            {
                m_odometryBufferMutex.lock();

                OdometryPtr interpOdo;
                if (m_poseSource == ODOMETRY && !m_interpOdometryBuffer.find(timeStamp, interpOdo))
                {
                    double timeStart = timeInSeconds();
                    while (!interpolateOdometry(m_odometryBuffer, timeStamp, interpOdo))
                    {
                        if (timeInSeconds() - timeStart > k_odometryTimeout)
                        {
                            std::cout << "# ERROR: No odometry data for " << k_odometryTimeout << "s. Exiting..." << std::endl;
                            exit(1);
                        }

                        usleep(1000);
                    }

                    m_interpOdometryBuffer.push(timeStamp, interpOdo);
                }

                m_odometryBufferMutex.unlock();

                m_gpsInsBufferMutex.lock();

                PosePtr interpGpsIns;
                if ((m_poseSource == GPS_INS || !m_gpsInsBuffer.empty()) && !m_interpGpsInsBuffer.find(timeStamp, interpGpsIns))
                {
                    double timeStart = timeInSeconds();
                    while (!interpolatePose(m_gpsInsBuffer, timeStamp, interpGpsIns))
                    {
                        if (timeInSeconds() - timeStart > k_odometryTimeout)
                        {
                            std::cout << "# ERROR: No GPS/INS data for " << k_odometryTimeout << "s. Exiting..." << std::endl;
                            exit(1);
                        }

                        usleep(1000);
                    }

                    printf("LOC: %f %f %f\n", interpGpsIns->translation()[0], interpGpsIns->translation()[1], interpGpsIns->translation()[2]);
                    m_interpGpsInsBuffer.push(timeStamp, interpGpsIns);
                }

                m_gpsInsBufferMutex.unlock();

                Eigen::Vector3d pos;
                if (m_poseSource == ODOMETRY)
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
                    (pos - framePrev->systemPose()->position()).norm() < k_minKeyframeDistance)
                {
                    m_image->notifyProcessingDone();
                    continue;
                }

                /*if (framePrev.get())
                {
                    static boost::mutex mmutex;
                    boost::mutex::scoped_lock lock(mmutex);

                    std::cout << "FRAME " << timeStamp << " -> mov=" << (pos - framePrev->systemPose()->position()).norm()
                              << ", POS=(" << pos.transpose() << ") PREVPOS=(" << framePrev->systemPose()->position().transpose() << ")" << std::endl;
                }*/

                FramePtr frame = boost::make_shared<Frame>();
                frame->cameraId() = m_cameraId;
                image.copyTo(frame->image());

                bool camValid = tracker.addFrame(frame, m_camera->mask());

                // tag frame with odometry and GPS/INS data

                if (interpOdo)
                {
                    frame->odometryMeasurement() = boost::make_shared<Odometry>();
                    *(frame->odometryMeasurement()) = *interpOdo;
                    frame->systemPose() = boost::make_shared<Odometry>();
                    *(frame->systemPose()) = *interpOdo;
                }

                if (interpGpsIns)
                {
                    frame->gpsInsMeasurement() = interpGpsIns;
                }

                if (m_poseSource == GPS_INS)
                {
                    OdometryPtr gpsIns = boost::make_shared<Odometry>();
                    gpsIns->timeStamp() = interpGpsIns->timeStamp();
                    gpsIns->x() = interpGpsIns->translation()(1);
                    gpsIns->y() = -interpGpsIns->translation()(0);
                    gpsIns->z() = interpGpsIns->translation()(2);

                    Eigen::Matrix3d R = interpGpsIns->rotation().toRotationMatrix();
                    double roll, pitch, yaw;
                    mat2RPY(R, roll, pitch, yaw);
                    gpsIns->yaw() = -yaw;

                    frame->odometryMeasurement() = boost::make_shared<Odometry>();
                    *(frame->odometryMeasurement()) = *gpsIns;
                    frame->systemPose() = boost::make_shared<Odometry>();
                    *(frame->systemPose()) = *gpsIns;
                }

                frame->cameraPose()->timeStamp() = timeStamp;

                if (camValid)
                {
                    odometryPoses.push_back(frame->systemPose());
                }

                framePrev = frame;

                if (!camValid)
                {
                    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > voPoses = tracker.getPoses();

                    if (odometryPoses.size() >= k_minVOSegmentSize)
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
        visualizeMap(overlay, tracker);
#endif

        int currentMotionCount = 0;
        if (odometryPoses.size() >= k_minVOSegmentSize)
        {
            currentMotionCount = odometryPoses.size() - 1;
        }

        // visualize feature tracks
        std::ostringstream oss;
        oss << "# motions: " << m_camOdoCalib.getCurrentMotionCount() + currentMotionCount << " | "
            << "# track breaks: " << trackBreaks;

        std::string status = oss.str();

        if (!tracker.getSketch().empty())
        {
            tracker.getSketch().copyTo(m_sketch);
        }
        else
        {
            colorImage.copyTo(m_sketch);
        }

        int fontFace = cv::FONT_HERSHEY_COMPLEX;
        double fontScale = 0.5;
        int thickness = 1;

        int baseline = 0;
        cv::Size textSize = cv::getTextSize(status, fontFace,
                                            fontScale, thickness, &baseline);
        baseline += thickness;

        // center the text horizontally and at bottom of image
        cv::Point textOrg((m_sketch.cols - textSize.width) / 2,
                           m_sketch.rows - textSize.height - 10);
        cv::putText(m_sketch, status, textOrg, fontFace, fontScale,
                    cv::Scalar::all(255), thickness, CV_AA);

        m_image->notifyProcessingDone();

        if (m_camOdoCalib.getCurrentMotionCount() + currentMotionCount >= m_camOdoCalib.getMotionCount())
        {
            m_completed = true;
        }
    }

    if (!m_camOdoTransformUseEstimate)
    {
    //    m_camOdoCalib.writeMotionSegmentsToFile(filename);

        Eigen::Matrix4d H_cam_odo;
        m_camOdoCalib.calibrate(H_cam_odo);

        m_camOdoTransform = H_cam_odo;
    }

    {
        static boost::mutex mutex;
        boost::mutex::scoped_lock lock(mutex);

        if (m_camOdoTransformUseEstimate)
            std::cout << "# INFO: Use provided odometry estimate for camera " << m_cameraId << "..." << std::endl;
        else
            std::cout << "# INFO: Calibrating odometry - camera " << m_cameraId << "..." << std::endl;

        std::cout << "Rotation: " << std::endl << m_camOdoTransform.block<3,3>(0,0) << std::endl;
        std::cout << "Translation: " << std::endl << m_camOdoTransform.block<3,1>(0,3).transpose() << std::endl;
    }

    m_running = false;

    m_signalFinished();
}

void
CamOdoThread::addCamOdoCalibData(const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> >& camPoses,
                                 const std::vector<OdometryPtr>& odoPoses,
                                 std::vector<FramePtr>& frameSegment)
{
    if (odoPoses.size() != camPoses.size())
    {
        std::cout << "# WARNING: Numbers of odometry (" << odoPoses.size()
                  << ") and camera poses (" << camPoses.size() << ") differ. Aborting..." << std::endl;

        return;
    }

    if (odoPoses.size() < k_minVOSegmentSize)
    {
        std::cout << "# WARNING: At least " << k_minVOSegmentSize << " poses are needed. Aborting..." << std::endl;

        return;
    }

    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > odoMotions;
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > camMotions;

    //static boost::mutex m;
    //boost::mutex::scoped_lock lock(m);

    for (size_t i = 1; i < odoPoses.size(); ++i)
    {
        Eigen::Matrix4d relativeOdometryPose = odoPoses.at(i)->toMatrix().inverse() * odoPoses.at(i - 1)->toMatrix();
        odoMotions.push_back(relativeOdometryPose);

        Eigen::Matrix4d relativeCameraPose = camPoses.at(i) * camPoses.at(i - 1).inverse();
        camMotions.push_back(relativeCameraPose);

        //Eigen::Vector3d todo = relativeOdometryPose.block<3,1>(0,3);
        //Eigen::Vector3d tcam = relativeCameraPose.block<3,1>(0,3);
        //if (std::isnan(todo[0]) || std::isnan(todo[1]) || std::isnan(todo[2]))
        //    std::cout << "odo [" << i << "] -> " << relativeOdometryPose.block<3,1>(0,3).transpose() << std::endl;
        //if (std::isnan(tcam[0]) || std::isnan(tcam[1]) || std::isnan(tcam[2]))
        //    std::cout << "cam [" << i << "] -> " << relativeCameraPose.block<3,1>(0,3).transpose() << std::endl;
    }

    if (!m_camOdoCalib.addMotionSegment(camMotions, odoMotions))
    {
        std::cout << "# ERROR: Numbers of odometry and camera motions do not match." << std::endl;
        exit(0);
    }

    m_frameSegments.push_back(frameSegment);
}

#ifdef VCHARGE_VIZ

void
CamOdoThread::visualizeMap(vcharge::GLOverlayExtended& overlay,
                           TemporalFeatureTracker& tracker)
{
    // visualize camera poses and 3D scene points
    const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> >& poses = tracker.getPoses();

    overlay.clear();
    overlay.pointSize(2.0f);
    overlay.lineWidth(1.0f);

    // draw 3D scene points
//    switch (cameraId)
//    {
//    case CAMERA_FRONT:
//        overlay.color4f(1.0f, 0.0f, 0.0f, 0.5f);
//        break;
//    case CAMERA_LEFT:
//        overlay.color4f(0.0f, 1.0f, 0.0f, 0.5f);
//        break;
//    case CAMERA_REAR:
//        overlay.color4f(0.0f, 0.0f, 1.0f, 0.5f);
//        break;
//    case CAMERA_RIGHT:
//        overlay.color4f(1.0f, 1.0f, 0.0f, 0.5f);
//        break;
//    default:
//        overlay.color4f(1.0f, 1.0f, 1.0f, 0.5f);
//    }
//
//    overlay.begin(VCharge::POINTS);
//
//    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > scenePoints = tracker.getScenePoints();
//    for (size_t j = 0; j < scenePoints.size(); ++j)
//    {
//        Eigen::Vector3d& p = scenePoints.at(j);
//
//        overlay.vertex3f(p(2), -p(0), -p(1));
//    }
//
//    overlay.end();

    // draw cameras
    for (size_t i = 0; i < poses.size(); ++i)
    {
        Eigen::Matrix4d H = poses.at(i).inverse();

        double xBound = 0.1;
        double yBound = 0.1;
        double zFar = 0.2;

        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > frustum;
        frustum.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
        frustum.push_back(Eigen::Vector3d(-xBound, -yBound, zFar));
        frustum.push_back(Eigen::Vector3d(xBound, -yBound, zFar));
        frustum.push_back(Eigen::Vector3d(xBound, yBound, zFar));
        frustum.push_back(Eigen::Vector3d(-xBound, yBound, zFar));

        for (size_t j = 0; j < frustum.size(); ++j)
        {
            frustum.at(j) = transformPoint(H, frustum.at(j));
        }

        overlay.color4f(1.0f, 1.0f, 1.0f, 1.0f);
        overlay.begin(VCharge::LINES);

        for (int j = 1; j < 5; ++j)
        {
            overlay.vertex3f(frustum.at(0)(2), -frustum.at(0)(0), -frustum.at(0)(1));
            overlay.vertex3f(frustum.at(j)(2), -frustum.at(j)(0), -frustum.at(j)(1));
        }

        overlay.end();

        switch (m_cameraId)
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

        for (int j = 1; j < 5; ++j)
        {
            overlay.vertex3f(frustum.at(j)(2), -frustum.at(j)(0), -frustum.at(j)(1));
        }

        overlay.end();
    }

    overlay.publish();
}

#endif

}
