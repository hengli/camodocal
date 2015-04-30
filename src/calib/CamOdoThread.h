#ifndef CAMODOTHREAD_H
#define CAMODOTHREAD_H

#include <boost/thread.hpp>
#include <boost/signals2.hpp>

#include "camodocal/calib/AtomicData.h"
#include "camodocal/calib/CamOdoCalibration.h"
#include "camodocal/calib/PoseSource.h"
#include "camodocal/calib/SensorDataBuffer.h"
#include "camodocal/camera_models/Camera.h"
#include "camodocal/sparse_graph/SparseGraph.h"

#ifdef VCHARGE_VIZ
#include "../../../../visualization/overlay/GLOverlayExtended.h"
#include "../visual_odometry/FeatureTracker.h"
#endif

namespace camodocal
{

class CamOdoThread
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CamOdoThread(PoseSource poseSource, int nMotions, int cameraId,
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
                 bool verbose = false);
    virtual ~CamOdoThread();

    int cameraId(void) const;
    const Eigen::Matrix4d& camOdoTransform(void) const;
    const std::vector<std::vector<FramePtr> >& frameSegments(void) const;

    void setCamOdoTransformEstimate(const Eigen::Matrix4d& estimate);
    void clearCamOdoTransformEstimate();
    void reprojectionError(double& minError, double& maxError, double& avgError) const;

    void launch(void);
    void join(void);
    bool running(void) const;
    boost::signals2::signal<void ()>& signalFinished(void);

private:
    void threadFunction(void);

    void addCamOdoCalibData(const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> >& camPoses,
                            const std::vector<OdometryPtr>& odoPoses,
                            std::vector<FramePtr>& frameSegment);

#ifdef VCHARGE_VIZ
    void visualizeMap(vcharge::GLOverlayExtended& overlay,
                      TemporalFeatureTracker& tracker);
#endif

    PoseSource m_poseSource;

    boost::shared_ptr<boost::thread> m_thread;
    int m_cameraId;
    bool m_preprocess;
    volatile bool m_running; // poor man's synchronisation
    boost::signals2::signal<void ()> m_signalFinished;

    CamOdoCalibration m_camOdoCalib;
    std::vector<std::vector<FramePtr> > m_frameSegments;

    AtomicData<cv::Mat>* m_image;
    const CameraConstPtr m_camera;
    SensorDataBuffer<OdometryPtr>& m_odometryBuffer;
    SensorDataBuffer<OdometryPtr>& m_interpOdometryBuffer;
    boost::mutex& m_odometryBufferMutex;
    SensorDataBuffer<PosePtr>& m_gpsInsBuffer;
    SensorDataBuffer<PosePtr>& m_interpGpsInsBuffer;
    boost::mutex& m_gpsInsBufferMutex;
    Eigen::Matrix4d m_camOdoTransform;
    bool m_camOdoTransformUseEstimate;
    cv::Mat& m_sketch;

    bool& m_completed;
    bool& m_stop;

    const double k_minKeyframeDistance;
    const size_t k_minVOSegmentSize;
    const double k_odometryTimeout;
};

}

#endif
