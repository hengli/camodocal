#ifndef CAMRIGODOCALIBRATION_H
#define CAMRIGODOCALIBRATION_H

#include <boost/multi_array.hpp>
#include <glibmm.h>

#include "camodocal/calib/AtomicData.h"
#include "camodocal/calib/PoseSource.h"
#include "camodocal/calib/SensorDataBuffer.h"
#include "camodocal/camera_models/Camera.h"
#include "camodocal/camera_systems/CameraRigExtrinsics.h"
#include "camodocal/sparse_graph/SparseGraph.h"

namespace camodocal
{

// forward declarations
class CamOdoThread;
class CamOdoWatchdogThread;
class CamRigThread;

class CamRigOdoCalibration: public sigc::trackable
{
public:
    enum Mode
    {
        OFFLINE,
        ONLINE
    };

    class Options
    {
    public:
        Options()
         : mode(OFFLINE)
         , poseSource(ODOMETRY)
         , nMotions(200)
         , findLoopClosures(true)
         , saveWorkingData(true)
         , beginStage(0)
         , optimizeIntrinsics(true)
         , saveImages(false)
         , verbose(false) {};

        Mode mode;
        PoseSource poseSource;
        int nMotions;

        bool findLoopClosures;
        bool saveWorkingData;
        int beginStage;
        bool optimizeIntrinsics;
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

    void start(void);
    void run(void);

    bool isRunning(void) const;

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
    SensorDataBuffer<OdometryPtr> mOdometryBuffer;
    SensorDataBuffer<OdometryPtr> mInterpOdometryBuffer;
    boost::mutex mOdometryBufferMutex;
    SensorDataBuffer<PosePtr> mGpsInsBuffer;
    SensorDataBuffer<PosePtr> mInterpGpsInsBuffer;
    boost::mutex mGpsInsBufferMutex;

    boost::multi_array<bool, 1> mCamOdoCompleted;

    std::vector<std::string> mStatuses;
    std::vector<cv::Mat> mSketches;

    Options mOptions;

    bool mRunning;
    static bool mStop;
};

}

#endif
