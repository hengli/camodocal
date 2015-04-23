#ifndef CAMRIGODOCALIBRATION_H
#define CAMRIGODOCALIBRATION_H

#include <boost/asio.hpp>
#include <boost/multi_array.hpp>

#include "camodocal/calib/AtomicData.h"
#include "camodocal/calib/PoseSource.h"
#include "camodocal/calib/SensorDataBuffer.h"
#include "camodocal/camera_systems/CameraSystem.h"
#include "camodocal/sparse_graph/SparseGraph.h"

namespace camodocal
{

// forward declarations
class CamOdoThread;
class CamOdoWatchdogThread;

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
         , minKeyframeDistance(0.2)
         , minVOSegmentSize(15)
         , windowDistance(3.0)
         , preprocessImages(false)
         , saveWorkingData(true)
         , beginStage(0)
         , optimizeIntrinsics(true)
         , verbose(false) {};

        Mode mode;
        PoseSource poseSource;
        int nMotions;            // Once we reach a number of keyframes for each camera
                                 // such that there are <nMotion> relative motions between
                                 // consecutive keyframes, the calibration runs automatically.

        // monocular VO
        double minKeyframeDistance; // Minimum distance between consecutive keyframes.
                                    // (Recommended: 0.2 m)
        size_t minVOSegmentSize;    // The VO segment will be used in calibration only if the number of
                                    // keyframes in the VO segment exceeds <minVOSegmentSize>.

        // local matching between cameras
        double windowDistance;   // The size of the window of frames in which local matching is
                                 // performed between different cameras depends on the
                                 // <windowDistance> distance that the vehicle travels
                                 // from the beginning of the window to the end of the window.
                                 // The larger the distance, the longer the local matching takes.

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

    void addOdometry(double x, double y, double yaw, uint64_t timestamp) { addOdometry(x,y,0,yaw,timestamp); }
    void addOdometry(double x, double y, double z, double yaw, uint64_t timestamp);

    void addGpsIns(double lat, double lon, double alt,
                   double roll, double pitch, double yaw,
                   uint64_t timestamp);
    void addGpsIns(double lat, double lon, double alt,
                   double qx, double qy, double qz, double qw,
                   uint64_t timestamp);

    //! If an initial odo transform estimate for a camera is specified there will be no automatic estimation step performed. (@note setup before start()!)
    void setInitialCameraOdoTransformEstimates(unsigned camIdx, const Eigen::Matrix4d& odoT);

    void start(void);
    void run(void);

    bool isRunning(void) const;

    const CameraSystem& cameraSystem(void) const;

private:
    void onCamOdoThreadFinished(CamOdoThread* odoCamThread);

    void buildGraph(void);

    void pollWindow(boost::asio::io_service* io, bool* stop);
    void displayHandler(boost::asio::deadline_timer* timer, bool* stop);

    std::vector<CamOdoThread*> m_camOdoThreads;
    CamOdoWatchdogThread* m_camOdoWatchdogThread;

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

    std::vector<cv::Mat> m_sketches;

    Options m_options;

    bool m_running;
    static bool m_stop;
};

}

#endif
