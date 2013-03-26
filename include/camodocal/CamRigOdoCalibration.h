#ifndef CAMRIGODOCALIBRATION_H
#define CAMRIGODOCALIBRATION_H

#include <glibmm.h>

#include "camodocal/CameraRigExtrinsics.h"
#include "camodocal/CamOdoCalibration.h"
#include "camodocal/CataCamera.h"
#include "camodocal/CataCameraCalibration.h"
#include "camodocal/SensorDataBuffer.h"
#include "camodocal/SparseGraph.h"

#include "AtomicData.h"

namespace camodocal
{

class CamOdoThread
{
public:
    explicit CamOdoThread(int nMotions, int cameraIdx,
                          AtomicData<cv::Mat>* image,
                          const CataCameraCalibration* const cameraCalib,
                          SensorDataBuffer<OdometerPtr>& odometerBuffer,
                          SensorDataBuffer<OdometerPtr>& interpOdometerBuffer,
                          boost::mutex& odometerBufferMutex,
                          SensorDataBuffer<PosePtr>& gpsInsBuffer,
                          SensorDataBuffer<PosePtr>& interpGpsInsBuffer,
                          boost::mutex& gpsInsBufferMutex,
                          std::string& status,
                          cv::Mat& sketch,
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

    Glib::Threads::Thread* mThread;
    int mCameraIdx;
    bool mRunning;
    sigc::signal<void> mSignalFinished;

    CamOdoCalibration mCamOdoCalib;
    std::vector<FrameSegment> mFrameSegments;

    AtomicData<cv::Mat>* mImage;
    const CataCameraCalibration* const mCameraCalib;
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

    bool& mStop;
    bool mSaveImages;
};

class CamRigThread
{
public:
    explicit CamRigThread(const std::vector<CataCameraCalibration*>& cameraCalib,
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

    const std::vector<CataCameraCalibration*>& mCameraCalib;
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
        Options() : nMotions(200), findLoopClosures(true), saveWorkingData(true), beginStage(0), saveImages(false), verbose(false) {};

        int nMotions;

        bool findLoopClosures;
        bool saveWorkingData;
        int beginStage;
        std::string dataDir;
        bool saveImages;
        bool verbose;
    };

    CamRigOdoCalibration(std::vector<CataCameraCalibration*>& cameraCalib,
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

    Glib::RefPtr<Glib::MainLoop> mMainLoop;
    std::vector<CamOdoThread*> mCamOdoThreads;
    CamRigThread* mCamRigThread;

    CameraRigExtrinsics mExtrinsics;
    SparseGraph mGraph;

    std::vector<AtomicData<cv::Mat>* > mImages;
    std::vector<CataCameraCalibration*> mCameraCalib;
    SensorDataBuffer<OdometerPtr> mOdometerBuffer;
    SensorDataBuffer<OdometerPtr> mInterpOdometerBuffer;
    boost::mutex mOdometerBufferMutex;
    SensorDataBuffer<PosePtr> mGpsInsBuffer;
    SensorDataBuffer<PosePtr> mInterpGpsInsBuffer;
    boost::mutex mGpsInsBufferMutex;

    std::vector<std::string> mStatuses;
    std::vector<cv::Mat> mSketches;

    Options mOptions;

    static bool mStop;
};

}

#endif
