#ifndef CAMRIGODOCALIBRATION_H
#define CAMRIGODOCALIBRATION_H

#include <boost/multi_array.hpp>

#include "camodocal/calib/AtomicData.h"
#include "camodocal/calib/PoseSource.h"
#include "camodocal/calib/SensorDataBuffer.h"
#include "camodocal/camera_systems/CameraSystem.h"
#include "camodocal/sparse_graph/SparseGraph.h"

#ifdef VCHARGE_VIZ
#include <boost/asio.hpp>
#endif

namespace camodocal
{

// forward declarations
class CamOdoThread;
class CamOdoWatchdogThread;
class CamRigThread;

class CamRigOdoCalibration
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
         , preprocessImages(false)
         , saveWorkingData(true)
         , beginStage(0)
         , optimizeIntrinsics(true)
         , verbose(false) {};

        Mode mode;
        PoseSource poseSource;
        int nMotions;

        bool preprocessImages;
        bool saveWorkingData;
        int beginStage;
        bool optimizeIntrinsics;
        std::string dataDir;
        bool verbose;
    };

    CamRigOdoCalibration(std::vector<CameraPtr>& cameras,
                         const Options& options);
    virtual ~CamRigOdoCalibration();

    void addFrame(int cameraIdx, const cv::Mat& image, uint64_t timestamp);
    void addFrameSet(const std::vector<cv::Mat>& images, uint64_t timestamp);

    void addOdometry(double x, double y, double yaw, uint64_t timestamp);

    void addGpsIns(double lat, double lon, double alt,
                   double roll, double pitch, double yaw,
                   uint64_t timestamp);

    void start(void);
    void run(void);

    bool isRunning(void) const;

    const CameraSystem& cameraSystem(void) const;

private:
    void onCamOdoThreadFinished(CamOdoThread* odoCamThread);

    void buildGraph(void);

#ifdef VCHARGE_VIZ
    void pollWindow(boost::asio::io_service* io, bool* stop);
    void displayHandler(boost::asio::deadline_timer* timer, bool* stop);
    static void keyboardHandler(unsigned char key, int x, int y);
#endif

    std::vector<CamOdoThread*> m_camOdoThreads;
    CamOdoWatchdogThread* m_camOdoWatchdogThread;
    CamRigThread* m_camRigThread;

    CameraSystem m_cameraSystem;
    SparseGraph m_graph;

    std::vector<AtomicData<cv::Mat>* > m_images;
    std::vector<CameraPtr> m_cameras;
    SensorDataBuffer<OdometryPtr> m_odometryBuffer;
    SensorDataBuffer<OdometryPtr> m_interpOdometryBuffer;
    boost::mutex m_odometryBufferMutex;
    SensorDataBuffer<PosePtr> m_gpsInsBuffer;
    SensorDataBuffer<PosePtr> m_interpGpsInsBuffer;
    boost::mutex m_gpsInsBufferMutex;

    boost::multi_array<bool, 1> m_camOdoCompleted;

    std::vector<std::string> m_statuses;
    std::vector<cv::Mat> m_sketches;

    Options m_options;

    bool m_running;
    static bool m_stop;
};

}

#endif
