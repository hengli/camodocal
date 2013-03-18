#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <iostream>
#include <Eigen/Eigen>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "CalibrationWindow.h"
#include "CameraRigCalibration.h"

void
imageHandler(void* msg, uint64_t timestamp, camodocal::AtomicData<cv::Mat>* frame)
{
    VCharge::Image* imageMsg = reinterpret_cast<VCharge::Image*>(msg);

    frame->lockData();

    frame->data() = cv::Mat(imageMsg->height, imageMsg->width, CV_8UC1,
                            imageMsg->data.get_contiguous_buffer(), imageMsg->step);

    frame->timeStamp() = timestamp;
    frame->available() = true;

    frame->unlockData();
}

void
odometerHandler(void* msg, uint64_t timestamp,
                vcharge::SensorDataBuffer<vcharge::OdometerPtr>* buffer)
{
    // odometer data is available on both Grobi and Kermit
    DDSEGONS::tEgoData* odometerMsg = reinterpret_cast<DDSEGONS::tEgoData*>(msg);

    vcharge::OdometerPtr odometer(new vcharge::Odometer);
    odometer->x() = odometerMsg->m_oEgoDynamic.f64X;
    odometer->y() = odometerMsg->m_oEgoDynamic.f64Y;
    odometer->yaw() = odometerMsg->m_oEgoDynamic.f64Yaw;

    odometer->timeStamp() = timestamp;

    buffer->push(timestamp, odometer);
}

void
gpsInsHandler(void* msg, uint64_t timestamp,
              vcharge::SensorDataBuffer<vcharge::PosePtr>* buffer)
{
    // GPS/INS data is available only on Grobi
    ITraceImu::ITraceImuData* gpsInsMsg = reinterpret_cast<ITraceImu::ITraceImuData*>(msg);

    // convert latitude/longitude to UTM coordinates
    double utmX, utmY;
    std::string utmZone;
    vcharge::LLtoUTM(gpsInsMsg->posLat_deg, gpsInsMsg->posLon_deg, utmX, utmY, utmZone);

    vcharge::PosePtr pose(new vcharge::Pose);
    pose->rotation() = Eigen::AngleAxisd(vcharge::d2r(gpsInsMsg->yaw_deg), Eigen::Vector3d::UnitZ())
                       * Eigen::AngleAxisd(vcharge::d2r(gpsInsMsg->pitch_deg), Eigen::Vector3d::UnitY())
                       * Eigen::AngleAxisd(vcharge::d2r(gpsInsMsg->roll_deg), Eigen::Vector3d::UnitX());
    pose->translation() = Eigen::Vector3d(utmX, utmY, 0.0);

    pose->timeStamp() = timestamp;

    buffer->push(timestamp, pose);
}

int
main(int argc, char** argv)
{
    using namespace vcharge;

    std::string calibDir;
    float focal;
    std::string outputFilename;
    int nMotions;
    int beginStage;
    bool findLoopClosures;
    std::string dataDir;
    bool saveImages;
    bool debug;
    bool verbose;

    //================= Handling Program options ==================
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("calib,c", boost::program_options::value<std::string>(&calibDir)->default_value("calib"), "Directory containing camera calibration files")
        ("f", boost::program_options::value<float>(&focal)->default_value(300.0f), "Nominal focal length")
        ("output,o", boost::program_options::value<std::string>(&outputFilename)->default_value("extrinsic.txt"), "Filename for extrinsic file to be written.")
        ("motions,m", boost::program_options::value<int>(&nMotions)->default_value(500), "Number of motions for calibration.")
        ("begin-stage", boost::program_options::value<int>(&beginStage)->default_value(0), "Stage to begin from")
        ("loop-closures", boost::program_options::bool_switch(&findLoopClosures)->default_value(false), "Find loop closures")
        ("data", boost::program_options::value<std::string>(&dataDir)->default_value("data"), "Location of folder which contains working data.")
        ("save-images", boost::program_options::bool_switch(&saveImages)->default_value(true), "Save images.")
        ("debug", boost::program_options::bool_switch(&debug)->default_value(false), "Debug mode")
        ("verbose,v", boost::program_options::bool_switch(&verbose)->default_value(false), "Verbose output")
        ;
    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return 1;
    }

    // Check if directory containing camera calibration files exists
    if (!boost::filesystem::exists(calibDir))
    {
        std::cout << "# ERROR: Directory " << calibDir << " does not exist." << std::endl;
        return 1;
    }

    // Check if extrinsic data file exists
    boost::filesystem::path extrinsicFilePath(calibDir + "/extrinsic.txt");
    if (!boost::filesystem::exists(extrinsicFilePath))
    {
        std::cout << "# ERROR: Camera extrinsic file " << extrinsicFilePath.string() << " does not exist." << std::endl;
        return 1;
    }

    std::cout << "# INFO: Initializing... " << std::flush;

    if (beginStage > 0)
    {
        // check for CUDA devices
        cv::gpu::DeviceInfo info;
        if (cv::gpu::getCudaEnabledDeviceCount() > 0 && info.isCompatible())
        {
            cv::gpu::setDevice(0);
            cv::gpu::resetDevice();

            // dummy function call
            cv::Mat dummy(1, 1, CV_8UC1);
            dummy = cv::Scalar(0);

            cv::gpu::GpuMat dummyGPU;
            dummyGPU.upload(dummy);

            dummyGPU.release();
        }
        else
        {
            std::cout << "# ERROR: No Cuda device found!\n";
            exit(0);
        }
    }

    //========================= Handling Input =======================
    Middleware mw;
    mw.init(argc, argv);

    std::vector<AtomicData<cv::Mat>* > frame(CAMERA_COUNT);
    SensorDataBuffer<OdometerPtr> odometerBuffer(1000);
    SensorDataBuffer<PosePtr> gpsInsBuffer(1000);
    if (beginStage == 0)
    {
        Handler handler;

        // subscribe to topics
        handler = Handler(sigc::bind(sigc::ptr_fun(odometerHandler), &odometerBuffer));
        OdometryEgoTopic::instance()->subscribe(handler, SUBSCRIBE_LATEST);

        handler = Handler(sigc::bind(sigc::ptr_fun(gpsInsHandler), &gpsInsBuffer));
        ITraceIMUTopic::instance()->subscribe(handler, SUBSCRIBE_LATEST);

        // advertise topics
        for (int i = 0; i < CAMERA_COUNT; ++i)
        {
            frame.at(i) = new AtomicData<cv::Mat>();

            handler = Handler(sigc::bind(sigc::ptr_fun(imageHandler), frame.at(i)));

            switch (i)
            {
            case CAMERA_FRONT:
                ImageMonoFrontTopic::instance()->subscribe(handler, SUBSCRIBE_LATEST);
                break;
            case CAMERA_LEFT:
                ImageMonoLeftTopic::instance()->subscribe(handler, SUBSCRIBE_LATEST);
                break;
            case CAMERA_REAR:
                ImageMonoRearTopic::instance()->subscribe(handler, SUBSCRIBE_LATEST);
                break;
            case CAMERA_RIGHT:
                ImageMonoRightTopic::instance()->subscribe(handler, SUBSCRIBE_LATEST);
                break;
            }
        }
    }
    vcharge::GLOverlayTopic::instance()->advertise();

    //===================== Initialize threading =====================
    // Only initialize g thread if not already done
    if (!Glib::thread_supported())
    {
        Glib::thread_init();
    }

    cv::Size frameSize(1280, 800);
    if (beginStage == 0)
    {
        std::cout << "# INFO: Waiting for image data from all cameras... " << std::flush;
        while (!frame.at(CAMERA_FRONT)->available() || !frame.at(CAMERA_LEFT)->available() ||
               !frame.at(CAMERA_REAR)->available() || !frame.at(CAMERA_RIGHT)->available())
        {
            usleep(100000);
        }
        std::cout << "Done." << std::endl;

        frameSize = frame.at(CAMERA_FRONT)->data().size();
    }

    //===========================Initialize calibration==========================
    
    // read catacamera params
    std::vector<CataCameraCalibration*> cameraCalib(CAMERA_COUNT);
    for (int i = 0; i < 4; ++i)
    {
        boost::filesystem::path calibFilePath(calibDir);
        boost::filesystem::path maskFilePath(calibDir);
        switch (i)
        {
        case CAMERA_FRONT:
            calibFilePath /= "mono_front_camera_calib.yaml";
            maskFilePath /= "mono_front_camera_mask.png";
            break;
        case CAMERA_LEFT:
            calibFilePath /= "mono_left_camera_calib.yaml";
            maskFilePath /= "mono_left_camera_mask.png";
            break;
        case CAMERA_REAR:
            calibFilePath /= "mono_rear_camera_calib.yaml";
            maskFilePath /= "mono_rear_camera_mask.png";
            break;
        case CAMERA_RIGHT:
            calibFilePath /= "mono_right_camera_calib.yaml";
            maskFilePath /= "mono_right_camera_mask.png";
            break;
        }

        CataCameraParameters cameraParams;
        if (!cameraParams.read(calibFilePath.string()))
        {
            std::cout << "# ERROR: Unable to read calibration file: " << calibFilePath.string() << std::endl;

            mw.shutdown();
            return 0;
        }

        cameraCalib.at(i) = new CataCameraCalibration("camera", frameSize);
        cameraCalib.at(i)->cameraParameters() = cameraParams;

        cv::Mat mask = cv::imread(maskFilePath.string(), CV_LOAD_IMAGE_GRAYSCALE);
        if (!mask.empty())
        {
            mask.copyTo(cameraCalib.at(i)->cameraMask());
        }
    }

    //========================= Start Threads =========================
    CameraRigCalibration::Options options;
    options.nMotions = nMotions;
    options.findLoopClosures = findLoopClosures;
    options.saveWorkingData = true;
    options.beginStage = beginStage;
    options.dataDir = dataDir;
    options.saveImages = saveImages;
    options.verbose = verbose;

    CameraRigCalibration cameraRigCalib(frame, cameraCalib, odometerBuffer, gpsInsBuffer, options);

    if (beginStage == 0)
    {
        CalibrationWindow::instance()->open("Calibration", frameSize.width, frameSize.height, 3);
    }

    std::cout << "# INFO: Initialization finished!" << std::endl;

    double tsStart = timeInSeconds();

    cameraRigCalib.run();

    std::cout << "# INFO: Calibration took " << timeInSeconds() - tsStart << "s." << std::endl;

    CameraRigExtrinsics extrinsics = cameraRigCalib.extrinsics();
    extrinsics.writeToFile(outputFilename);

    std::cout << "# INFO: Wrote extrinsic data to " << outputFilename << "." << std::endl;

    std::cout << std::fixed << std::setprecision(5);

    // read current data
    CameraRigExtrinsics extrinsicsPrev(CAMERA_COUNT);
    extrinsicsPrev.readFromFile(extrinsicFilePath.string());
    extrinsicsPrev.setReferenceCamera(CAMERA_FRONT);

    std::cout << "# INFO: Previous estimate (local):" << std::endl;
    for (int i = 0; i < CAMERA_COUNT; ++i)
    {
        Eigen::Matrix4d H = extrinsicsPrev.getLocalCameraPose(i);

        std::cout << "========== Camera " << i << " ==========" << std::endl;
        std::cout << "Rotation: " << std::endl;
        std::cout << H.block<3,3>(0,0) << std::endl;

        std::cout << "Translation: " << std::endl;
        std::cout << H.block<3,1>(0,3).transpose() << std::endl;
    }
    std::cout << "# INFO: Previous estimate (global):" << std::endl;
    for (int i = 0; i < CAMERA_COUNT; ++i)
    {
        Eigen::Matrix4d H = extrinsicsPrev.getGlobalCameraPose(i);

        std::cout << "========== Camera " << i << " ==========" << std::endl;
        std::cout << "Rotation: " << std::endl;
        std::cout << H.block<3,3>(0,0) << std::endl;

        std::cout << "Translation: " << std::endl;
        std::cout << H.block<3,1>(0,3).transpose() << std::endl;
    }

    extrinsics.setReferenceCamera(CAMERA_FRONT);

    std::cout << "# INFO: Current estimate (local):" << std::endl;
    for (int i = 0; i < CAMERA_COUNT; ++i)
    {
        const Eigen::Matrix4d& H = extrinsics.getLocalCameraPose(i);

        std::cout << "========== Camera " << i << " ==========" << std::endl;
        std::cout << "Rotation: " << std::endl;
        std::cout << H.block<3,3>(0,0) << std::endl;

        std::cout << "Translation: " << std::endl;
        std::cout << H.block<3,1>(0,3).transpose() << std::endl;
    }

    std::cout << "# INFO: Current estimate (global):" << std::endl;
    for (int i = 0; i < CAMERA_COUNT; ++i)
    {
        const Eigen::Matrix4d& H = extrinsics.getGlobalCameraPose(i);

        std::cout << "========== Camera " << i << " ==========" << std::endl;
        std::cout << "Rotation: " << std::endl;
        std::cout << H.block<3,3>(0,0) << std::endl;

        std::cout << "Translation: " << std::endl;
        std::cout << H.block<3,1>(0,3).transpose() << std::endl;
    }

    if (beginStage == 0)
    {
        CalibrationWindow::instance()->close();
    }

    mw.shutdown();

    return 0;
}
