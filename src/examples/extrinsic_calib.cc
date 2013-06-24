#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <iomanip>
#include <iostream>
#include <Eigen/Eigen>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "camodocal/calib/CamRigOdoCalibration.h"
#include "camodocal/camera_models/CameraFactory.h"

#include "aslam/data_parser/DataParser.hpp"
#include "aslam/data_parser/Measurements.hpp"
#include "aslam/data_parser/OdometryIntegrator.hpp"

#include <limits>

#include <chrono>

int
main(int argc, char** argv)
{
    using namespace camodocal;

    std::string calibDir;
    int cameraCount;
    float focal;
    std::string outputFilename;
    int nMotions;
    int beginStage;
    bool findLoopClosures;
    std::string dataDir;
    bool saveImages;
    bool debug;
    bool verbose;
    std::string bagFilename;
    std::string sensorConfiguration;


    //================= Handling Program options ==================
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("bag,b", boost::program_options::value<std::string>(&bagFilename)->default_value(""), "Bag file containing camera, GPS or odometry data.")
        ("sensor-configuration", boost::program_options::value<std::string>(&sensorConfiguration)->default_value(""), ".info file describing the sensor configuration and topics of the bag file.")
        ("calib,c", boost::program_options::value<std::string>(&calibDir)->default_value("calib"), "Directory containing camera calibration files")
        ("camera-count", boost::program_options::value<int>(&cameraCount)->default_value(1), "Number of cameras in rig")
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

        camodocal::CameraPtr camera = camodocal::CameraFactory::instance()->generateCamera(calibFilePath.string());
        if (camera.get() == 0)
        {
            std::cout << "# ERROR: Unable to read calibration file: " << calibFilePath.string() << std::endl;

            return 0;
        }

        cameras.at(i) = camera;
    }

    //========================= Start Threads =========================
    CamRigOdoCalibration::Options options;
    options.poseSource = ODOMETRY;
    options.nMotions = nMotions;
    options.findLoopClosures = findLoopClosures;
    options.saveWorkingData = true;
    options.beginStage = beginStage;
    options.dataDir = dataDir;
    options.saveImages = saveImages;
    options.verbose = verbose;

    if (bagFilename == "")
    {
     	std::cout << "# ERROR: No ROS bag file provided. Aborting!" << std::endl;
     	return 0;
    }
    if (sensorConfiguration == "")
    {
    	std::cout << "# ERROR: No sensor configuration for the ROS bag specified. Aborting" << std::endl;
    	return 0;
    }

    // parse BAG file
    aslam::backend::RosbagDataParser dataParser(bagFilename, sensorConfiguration);
    dataParser.parseAll();

	  std::vector<GPSMeasurement> gpsMeasurements = dataParser.getGPSMeasurements();
	  std::vector<OdometryFrontWheelMeasurement> frontWheelMeasurements = dataParser.getOdometryFrontWheelMeasurements();
	  std::vector<OdometryBackWheelMeasurement> backWheelMeasurements = dataParser.getOdometryBackWheelMeasurements();
	  std::vector<OdometrySteeringMeasurement> steeringMeasurements = dataParser.getOdometrySteeringMeasurements();
	  std::vector<CameraMeasurement> cameraImages = dataParser.getCameraMeasurements();

	  aslam::backend::OdometryIntegrator odoIntegrator;

	  double yaw0 = 0;
	  if (gpsMeasurements.size() > 0)
	  {
		  yaw0 = gpsMeasurements.begin()->_yaw;
	  }
	  std::vector<IntegratedOdometryMeasurement> integratedOdometry = odoIntegrator.integrateOdometry(frontWheelMeasurements, backWheelMeasurements, steeringMeasurements, yaw0);


    CamRigOdoCalibration camRigOdoCalib(cameras, options, integratedOdometry.size(), gpsMeasurements.size());

    std::cout << "# INFO: Initialization finished!" << std::endl;

    std::cout << "# INFO: Reading in sensor data..." << std::endl;

    unsigned long tms_first_odo, tms_last_odo, tms_odo, tms_gps, tms_cam;

    if (cameraImages.size() == 0)
    {
    	std::cout << "# ERROR: No camera images found. Aborting" << std::endl;
    	return 0;
    }

    if (integratedOdometry.size() < 2)
    {
    	std::cout << "# ERROR: Not enough odometry measurements provided. Aborting" << std::endl;
    	return 0;
    }

    tms_first_odo = static_cast<unsigned long>(std::chrono::duration_cast<std::chrono::milliseconds>(integratedOdometry[0]._timestamp.time_since_epoch()).count());
    tms_last_odo = static_cast<unsigned long>(std::chrono::duration_cast<std::chrono::milliseconds>(integratedOdometry[integratedOdometry.size() -1 ]._timestamp.time_since_epoch()).count());

    // read in odo
  	for(std::vector<IntegratedOdometryMeasurement>::const_iterator it = integratedOdometry.begin(); it != integratedOdometry.end(); ++it)
  	{
  		tms_odo = static_cast<unsigned long>(std::chrono::duration_cast<std::chrono::milliseconds>(it->_timestamp.time_since_epoch()).count());
  		camRigOdoCalib.addOdometry(it->_x, it->_y, it->_theta, tms_odo);
  		if (odoM++ == 0)
  		{
  			std::cout << "first odo timestamp: " << tms_odo << std::endl;
  		}
  	}
  	// read in GPS
//  	for(std::vector<GPSMeasurement>::const_iterator it = gpsMeasurements.begin(); it != gpsMeasurements.end(); ++it)
//  	{
//  		tms_gps = static_cast<unsigned long>(std::chrono::duration_cast<std::chrono::milliseconds>(it->_timestamp.time_since_epoch()).count());
//  		camRigOdoCalib.addGpsIns(it->_latitude, it->_longitude, it->_roll, it->_pitch, it->_yaw, tms_gps);
//  	}

  	int camFrames = 0;
  	//int maxNumCamFrames = 500;
	for(std::vector<CameraMeasurement>::const_iterator it = cameraImages.begin(); it != cameraImages.end(); ++it)
	{
		tms_cam = static_cast<unsigned long>(std::chrono::duration_cast<std::chrono::milliseconds>(it->_timestamp.time_since_epoch()).count());
		// don't add any camera frames before the first and after the last odometere frame (because we have no odometer extrapolation implemented (yet)
		if (tms_cam <= tms_first_odo) continue;
		if (tms_cam >= tms_odo) break;

		camRigOdoCalib.addFrameOffline(it->_cameraIndex, it->_image, tms_cam, it->_imageBuffer);
		camFrames++;
		//if (camFrames > maxNumCamFrames) break;
	}

    camRigOdoCalib.run(true);

    //****************
    //
    // IMPORTANT: Add data in the order of increasing timestamp.
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
    //****************

    CameraRigExtrinsics extrinsics = camRigOdoCalib.extrinsics();
    extrinsics.writeToFile(outputFilename);

    std::cout << "# INFO: Wrote extrinsic data to " << outputFilename << "." << std::endl;

    std::cout << std::fixed << std::setprecision(5);

    // read current data
    CameraRigExtrinsics extrinsicsPrev(cameraCount);
    extrinsicsPrev.readFromFile(extrinsicFilePath.string());
    extrinsicsPrev.setReferenceCamera(0);

    std::cout << "# INFO: Previous estimate (local):" << std::endl;
    for (int i = 0; i < cameraCount; ++i)
    {
        Eigen::Matrix4d H = extrinsicsPrev.getLocalCameraPose(i);

        std::cout << "========== Camera " << i << " ==========" << std::endl;
        std::cout << "Rotation: " << std::endl;
        std::cout << H.block<3,3>(0,0) << std::endl;

        std::cout << "Translation: " << std::endl;
        std::cout << H.block<3,1>(0,3).transpose() << std::endl;
    }
    std::cout << "# INFO: Previous estimate (global):" << std::endl;
    for (int i = 0; i < cameraCount; ++i)
    {
        Eigen::Matrix4d H = extrinsicsPrev.getGlobalCameraPose(i);

        std::cout << "========== Camera " << i << " ==========" << std::endl;
        std::cout << "Rotation: " << std::endl;
        std::cout << H.block<3,3>(0,0) << std::endl;

        std::cout << "Translation: " << std::endl;
        std::cout << H.block<3,1>(0,3).transpose() << std::endl;
    }

    extrinsics.setReferenceCamera(0);

    std::cout << "# INFO: Current estimate (local):" << std::endl;
    for (int i = 0; i < cameraCount; ++i)
    {
        const Eigen::Matrix4d& H = extrinsics.getLocalCameraPose(i);

        std::cout << "========== Camera " << i << " ==========" << std::endl;
        std::cout << "Rotation: " << std::endl;
        std::cout << H.block<3,3>(0,0) << std::endl;

        std::cout << "Translation: " << std::endl;
        std::cout << H.block<3,1>(0,3).transpose() << std::endl;
    }

    std::cout << "# INFO: Current estimate (global):" << std::endl;
    for (int i = 0; i < cameraCount; ++i)
    {
        const Eigen::Matrix4d& H = extrinsics.getGlobalCameraPose(i);

        std::cout << "========== Camera " << i << " ==========" << std::endl;
        std::cout << "Rotation: " << std::endl;
        std::cout << H.block<3,3>(0,0) << std::endl;

        std::cout << "Translation: " << std::endl;
        std::cout << H.block<3,1>(0,3).transpose() << std::endl;
    }

    return 0;
}
