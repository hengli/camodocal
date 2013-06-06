#ifndef CAMRIGODOCALIBRATION_H
#define CAMRIGODOCALIBRATION_H

#include <boost/multi_array.hpp>
#include <glibmm.h>

#include "camodocal/calib/AtomicData.h"
#include "camodocal/calib/CamOdoCalibration.h"
#include "camodocal/calib/SensorDataBuffer.h"
#include "camodocal/camera_models/Camera.h"
#include "camodocal/camera_systems/CameraRigExtrinsics.h"
#include "camodocal/sparse_graph/SparseGraph.h"

namespace camodocal
{

enum PoseSource
{
    GPS_INS,
    ODOMETRY
};

class CamOdoThread
{
public:
    explicit CamOdoThread(PoseSource poseSource, int nMotions, int cameraIdx,
                          AtomicData<cv::Mat>* image,
                          const CameraConstPtr& camera,
                          SensorDataBuffer<OdometerPtr>& odometerBuffer,
                          SensorDataBuffer<OdometerPtr>& interpOdometerBuffer,
                          boost::mutex& odometerBufferMutex,
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
                            const std::vector<OdometerPtr>& odoPoses,
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
    SensorDataBuffer<OdometerPtr>& mOdometerBuffer;
    SensorDataBuffer<OdometerPtr>& mInterpOdometerBuffer;
    boost::mutex& mOdometerBufferMutex;
    SensorDataBuffer<PosePtr>& mGpsInsBuffer;
    SensorDataBuffer<PosePtr>& mInterpGpsInsBuffer;
    boost::mutex& mGpsInsBufferMutex;
    Eigen::Matrix4d mCamOdoTransform;
    std::string& mStatus;
    cv::Mat& mSketch;

    const double kKeyFrameDistance;
    const int kMinTrackLength;
    const double kOdometerTimeout;

    bool& mCompleted;
    bool& mStop;
    bool mSaveImages;
};

class CamOdoWatchdogThread
{
public:
    explicit CamOdoWatchdogThread(boost::multi_array<bool, 1>& completed,
                                  bool& stop);
    virtual ~CamOdoWatchdogThread();

    void launch(void);
    void join(void);
    bool running(void) const;
    sigc::signal<void>& signalFinished(void);

private:
    void threadFunction(void);

    Glib::Threads::Thread* mThread;
    bool mRunning;
    sigc::signal<void> mSignalFinished;

    boost::multi_array<bool, 1>& mCompleted;
    bool& mStop;
};

class CamRigThread
{
public:
    explicit CamRigThread(const std::vector<CameraPtr>& cameras,
                          CameraRigExtrinsics& cameraRigExt,
                          SparseGraph& graph,
                          int beginStage = 1,
                          bool findLoopClosures = true,
                          bool saveWorkingData = false,
                          std::string dataDir = "data",
                          bool verbose = false);
    virtual ~CamRigThread();

    void launch(void);
    void join(void);
    bool running(void) const;
    sigc::signal<void>& signalFinished(void);

private:
    void threadFunction(void);

    Glib::Threads::Thread* mThread;
    bool mRunning;
    sigc::signal<void> mSignalFinished;

    const std::vector<CameraPtr>& mCameras;
    CameraRigExtrinsics& mCameraRigExt;
    SparseGraph& mGraph;

    int mBeginStage;
    bool mFindLoopClosures;
    bool mSaveWorkingData;
    std::string mDataDir;
    bool mVerbose;
};

class CamRigOdoCalibration: public sigc::trackable
{
public:
    class Options
    {
    public:
        Options() : poseSource(ODOMETRY), nMotions(200), findLoopClosures(true), saveWorkingData(true), beginStage(0), saveImages(false), verbose(false) {};

        PoseSource poseSource;
        int nMotions;

        bool findLoopClosures;
        bool saveWorkingData;
        int beginStage;
        std::string dataDir;
        bool saveImages;
        bool verbose;
    };

    CamRigOdoCalibration(std::vector<CameraPtr>& cameras,
                         const Options& options);
    virtual ~CamRigOdoCalibration();

    void addFrame(int cameraIdx, const cv::Mat& image, uint64_t timestamp);

    void addOdometry(double x, double y, double yaw, uint64_t timestamp);

    void addGpsIns(double lat, double lon,
                   double roll, double pitch, double yaw,
                   uint64_t timestamp);

    void run(void);

    const CameraRigExtrinsics& extrinsics(void) const;

private:
    void launchCamOdoThreads(void);

    void onCamOdoThreadFinished(CamOdoThread* odoCamThread);
    void onCamRigThreadFinished(CamRigThread* camRigThread);

    bool displayHandler(void);
    static void keyboardHandler(unsigned char key, int x, int y);

    Glib::RefPtr<Glib::MainLoop> mMainLoop;
    std::vector<CamOdoThread*> mCamOdoThreads;
    CamOdoWatchdogThread* mCamOdoWatchdogThread;
    CamRigThread* mCamRigThread;

    CameraRigExtrinsics mExtrinsics;
    SparseGraph mGraph;

    std::vector<AtomicData<cv::Mat>* > mImages;
    std::vector<CameraPtr> mCameras;
    SensorDataBuffer<OdometerPtr> mOdometerBuffer;
    SensorDataBuffer<OdometerPtr> mInterpOdometerBuffer;
    boost::mutex mOdometerBufferMutex;
    SensorDataBuffer<PosePtr> mGpsInsBuffer;
    SensorDataBuffer<PosePtr> mInterpGpsInsBuffer;
    boost::mutex mGpsInsBufferMutex;

    boost::multi_array<bool, 1> mCamOdoCompleted;

    std::vector<std::string> mStatuses;
    std::vector<cv::Mat> mSketches;

    Options mOptions;

    static bool mStop;
};

}

#endif
