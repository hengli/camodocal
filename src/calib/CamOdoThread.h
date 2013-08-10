#ifndef CAMODOTHREAD_H
#define CAMODOTHREAD_H

#include <glibmm.h>

#include "camodocal/calib/AtomicData.h"
#include "camodocal/calib/CamOdoCalibration.h"
#include "camodocal/calib/PoseSource.h"
#include "camodocal/calib/SensorDataBuffer.h"
#include "camodocal/camera_models/Camera.h"
#include "camodocal/sparse_graph/SparseGraph.h"

namespace camodocal
{

class CamOdoThread
{
public:
    explicit CamOdoThread(PoseSource poseSource, int nMotions, int cameraIdx,
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
                          bool saveImages = false,
                          bool verbose = false);
    virtual ~CamOdoThread();

    int cameraIdx(void) const;
    const Eigen::Matrix4d& camOdoTransform(void) const;
    const std::vector<FrameSegment>& frameSegments(void) const;

    void reprojectionError(double& minError, double& maxError, double& avgError) const;

    void launch(void);
    void join(void);
    bool running(void) const;
    sigc::signal<void>& signalFinished(void);

private:
    void threadFunction(void);

    void addCamOdoCalibData(const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> >& camPoses,
                            const std::vector<OdometryPtr>& odoPoses,
                            FrameSegment& frameSegment);

    PoseSource mPoseSource;

    Glib::Threads::Thread* mThread;
    int mCameraIdx;
    bool mRunning;
    sigc::signal<void> mSignalFinished;

    CamOdoCalibration mCamOdoCalib;
    std::vector<FrameSegment> mFrameSegments;

    AtomicData<cv::Mat>* mImage;
    const CameraConstPtr mCamera;
    SensorDataBuffer<OdometryPtr>& mOdometryBuffer;
    SensorDataBuffer<OdometryPtr>& mInterpOdometryBuffer;
    boost::mutex& mOdometryBufferMutex;
    SensorDataBuffer<PosePtr>& mGpsInsBuffer;
    SensorDataBuffer<PosePtr>& mInterpGpsInsBuffer;
    boost::mutex& mGpsInsBufferMutex;
    Eigen::Matrix4d mCamOdoTransform;
    std::string& mStatus;
    cv::Mat& mSketch;

    const double kKeyFrameDistance;
    const int kMinTrackLength;
    const double kOdometryTimeout;

    bool& mCompleted;
    bool& mStop;
    bool mSaveImages;
};

}

#endif
