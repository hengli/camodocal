#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <iomanip>
#include <iostream>
#include <Eigen/Eigen>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "camodocal/calib/CamRigOdoCalibration.h"
#include "camodocal/camera_models/CameraFactory.h"

int
main(int argc, char** argv)
{
    using namespace camodocal;

    std::string calibDir;
    int cameraCount;
    float focal;
    std::string outputDir;
    int nMotions;
    int beginStage;
    bool preprocessImages;
    bool optimizeIntrinsics;
    std::string dataDir;
    bool verbose;

    //================= Handling Program options ==================
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("calib,c", boost::program_options::value<std::string>(&calibDir)->default_value("calib"), "Directory containing camera calibration files.")
        ("camera-count", boost::program_options::value<int>(&cameraCount)->default_value(1), "Number of cameras in rig.")
        ("f", boost::program_options::value<float>(&focal)->default_value(300.0f), "Nominal focal length.")
        ("output,o", boost::program_options::value<std::string>(&outputDir)->default_value("calibration_data"), "Directory to write calibration data to.")
        ("motions,m", boost::program_options::value<int>(&nMotions)->default_value(500), "Number of motions for calibration.")
        ("begin-stage", boost::program_options::value<int>(&beginStage)->default_value(0), "Stage to begin from.")
        ("preprocess", boost::program_options::bool_switch(&preprocessImages)->default_value(false), "Preprocess images.")
        ("optimize-intrinsics", boost::program_options::bool_switch(&optimizeIntrinsics)->default_value(false), "Optimize intrinsics in BA step.")
        ("data", boost::program_options::value<std::string>(&dataDir)->default_value("data"), "Location of folder which contains working data.")
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

    //===================== Initialize threading =====================
    // Only initialize g thread if not already done
    if (!Glib::thread_supported())
    {
        Glib::thread_init();
    }

    //===========================Initialize calibration==========================

    // read camera params
    std::vector<camodocal::CameraPtr> cameras(cameraCount);
    for (int i = 0; i < cameraCount; ++i)
    {
        boost::filesystem::path calibFilePath(calibDir);

        std::ostringstream oss;
        oss << "camera_" << i << "_calib.yaml";
        calibFilePath /= oss.str();

        camodocal::CameraPtr camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(calibFilePath.string());
        if (camera.get() == 0)
        {
            std::cout << "# ERROR: Unable to read calibration file: " << calibFilePath.string() << std::endl;

            return 0;
        }

        cameras.at(i) = camera;
    }

    //========================= Start Threads =========================


    // optimize intrinsics only if features are well distributed across
    // the entire image area.
    CamRigOdoCalibration::Options options;
//    options.mode = CamRigOdoCalibration::ONLINE;
    options.poseSource = ODOMETRY;
    options.nMotions = nMotions;
    options.preprocessImages = preprocessImages;
    options.optimizeIntrinsics = optimizeIntrinsics;
    options.saveWorkingData = true;
    options.beginStage = beginStage;
    options.dataDir = dataDir;
    options.verbose = verbose;

    CamRigOdoCalibration camRigOdoCalib(cameras, options);

    std::cout << "# INFO: Initialization finished!" << std::endl;

    //****************
    //
    // IMPORTANT: Create a thread, and in this thread,
    //            add data in the order of increasing timestamp
    //            with one important exception for offline mode:
    //            ensure that before you add a frame with timestamp t,
    //            you have already added either odometry or GPS/INS data
    //            with a timestamp greater than t, depending on the
    //            pose source you are calibrating against.
    //
    // Add odometry and image data here.
    // camRigOdoCalib.addOdometry(x, y, yaw, timestamp);
    // camRigOdoCalib.addFrame(cameraId, image, timestamp);
    //
    // Alternatively, if you are calibrating against GPS/INS,
    // set options.poseSource = GPS_INS, and add GPS/INS
    // and image data here.
    //
    // camRigOdoCalib.addGpsIns(lat, lon, roll, pitch, yaw, timestamp);
    // camRigOdoCalib.addFrame(cameraId, image, timestamp);
    //
    // If options.mode == CamRigOdoCalibration::ONLINE,
    // the addFrame call returns immediately.
    // If options.mode == CamRigOdoCalibration::OFFLINE,
    // the addFrame call returns after the image has been processed.
    //
    // After you are done, if the minimum number of motions has not been
    // reached, but you want to run the calibration anyway, call:
    // camRigOdoCalib.run();
    //
    //****************

    // Receive and process incoming data. Calibration automatically runs
    // once minimum number of motions has been reached for all cameras.
    // Check camRigOdoCalib.running() to see if the calibration is running.
    // If so, you can stop adding data. To run the calibration without
    // waiting for the minimum motion requirement to be met,
    // call camRigOdoCalib.run().
    camRigOdoCalib.start();

    CameraSystem cameraSystem = camRigOdoCalib.cameraSystem();
    cameraSystem.writeToDirectory(outputDir);

    std::cout << "# INFO: Wrote calibration data to " << outputDir << "." << std::endl;

    std::cout << std::fixed << std::setprecision(5);

    std::cout << "# INFO: Current estimate (local):" << std::endl;
    for (int i = 0; i < cameraCount; ++i)
    {
        const Eigen::Matrix4d& H = cameraSystem.getLocalCameraPose(i);

        std::cout << "========== Camera " << i << " ==========" << std::endl;
        std::cout << "Rotation: " << std::endl;
        std::cout << H.block<3,3>(0,0) << std::endl;

        std::cout << "Translation: " << std::endl;
        std::cout << H.block<3,1>(0,3).transpose() << std::endl;
    }

    std::cout << "# INFO: Current estimate (global):" << std::endl;
    for (int i = 0; i < cameraCount; ++i)
    {
        const Eigen::Matrix4d& H = cameraSystem.getGlobalCameraPose(i);

        std::cout << "========== Camera " << i << " ==========" << std::endl;
        std::cout << "Rotation: " << std::endl;
        std::cout << H.block<3,3>(0,0) << std::endl;

        std::cout << "Translation: " << std::endl;
        std::cout << H.block<3,1>(0,3).transpose() << std::endl;
    }

    return 0;
}
