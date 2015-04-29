#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/thread.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include <iomanip>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <stdexcept>
#include <sstream>
#include <Eigen/Eigen>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "camodocal/calib/CamRigOdoCalibration.h"
#include "camodocal/camera_models/CameraFactory.h"

struct OdometryData
{
  uint64_t time;
  double x;
  double y;
  double theta;

};

struct GpsData
{
  uint64_t time;
  double lat;
  double lon;
  double alt;
  double roll;
  double pitch;
  double yaw;
};

class DataFeeder
{
 public:
  DataFeeder(
      camodocal::CamRigOdoCalibration* camOdoCal,
      const std::string& dataDirectory)
       : _camOdoCal(camOdoCal), _dataDirectory(dataDirectory) {};
  virtual ~DataFeeder() {};

  void addData(bool useGps);

 private:
  void readImageTimes(std::vector<uint64_t>& timeData);
  void readOdometryFile(std::vector<OdometryData>& odoData);
  void readGPSFile(std::vector<GpsData>& gpsData);


  camodocal::CamRigOdoCalibration* _camOdoCal;
  std::string _dataDirectory;

};

void DataFeeder::readImageTimes(std::vector<uint64_t>& timeData)
{
  std::string imageTimesFile = _dataDirectory + "/imageTimes.txt";
  std::cout << "opening file: " << imageTimesFile << std::endl;

  std::ifstream timeFs(imageTimesFile.c_str());
  std::string line;
  if(timeFs.is_open())
  {
    while (!timeFs.eof())
    {
      std::vector<std::string> tkns;
      timeFs >> line;

      uint64_t t = boost::lexical_cast<uint64_t>(line);

      timeData.push_back(t);
     }
     timeFs.close();
  } else
  {
    throw std::runtime_error("cannot open time data file");
  }
}

void DataFeeder::readOdometryFile(std::vector<OdometryData>& odoData)
{
  std::string odometryFile = _dataDirectory + "/odometry.txt";
  std::cout << "opening file: " << odometryFile << std::endl;

  std::ifstream odoFs(odometryFile.c_str());
  std::string line;
  if(odoFs.is_open())
  {
    while (!odoFs.eof())
    {
      std::vector<std::string> tkns;
      odoFs >> line;
      boost::split(tkns, line, boost::is_any_of(";"));
      if(tkns.size() != 4)
      {
        odoFs.close();
        std::cout << "tokens:" << std::endl;
        for(size_t i = 0; i < tkns.size(); i++)
        {
          std::cout << tkns[i] << std::endl;
        }
        throw std::runtime_error("bad input");
      }
      OdometryData odo;

      odo.time = boost::lexical_cast<uint64_t>(tkns[0]);
      odo.x = boost::lexical_cast<double>(tkns[1]);
      odo.y = boost::lexical_cast<double>(tkns[2]);
      odo.theta = boost::lexical_cast<double>(tkns[3]);

      odoData.push_back(odo);
     }
     odoFs.close();
  } else
  {
    throw std::runtime_error("cannot open odometry file");
  }
}

void DataFeeder::readGPSFile(std::vector<GpsData>& gpsData)
{
  std::string gpsFile = _dataDirectory + "/gps.txt";
  std::cout << "opening file: " << gpsFile << std::endl;

  std::ifstream gpsFs(gpsFile.c_str());
  std::string line;
  if(gpsFs.is_open())
  {
    while (!gpsFs.eof())
    {
      std::vector<std::string> tkns;
      gpsFs >> line;
      //std::cout << "read line: " << line << std::endl;
      boost::split(tkns, line, boost::is_any_of(";"));
      if(tkns.size() != 7)
      {
        gpsFs.close();
        throw std::runtime_error("bad input");
      }
      GpsData gps;

      gps.time = boost::lexical_cast<uint64_t>(tkns[0]);
      gps.lat = boost::lexical_cast<double>(tkns[1]);
      gps.lon = boost::lexical_cast<double>(tkns[2]);
      gps.alt = boost::lexical_cast<double>(tkns[3]);
      gps.roll = boost::lexical_cast<double>(tkns[4]);
      gps.pitch = boost::lexical_cast<double>(tkns[5]);
      gps.yaw = boost::lexical_cast<double>(tkns[6]);

      gpsData.push_back(gps);
   }
   gpsFs.close();
  } else
  {
    throw std::runtime_error("cannot open odometry file");
  }
}

void DataFeeder::addData(bool gps)
{
  if(!gps) throw std::runtime_error("odometry not supported!");
  if(_camOdoCal == 0) throw std::runtime_error("camodocal is NULL");

  std::string imageDir = _dataDirectory;

  std::vector<OdometryData> odoData;
  //readOdometryFile(odoData);

  std::vector<GpsData> gpsData;
  readGPSFile(gpsData);

  //if(odoData.size() < 1) throw std::runtime_error("no odo data!");
  if(gpsData.size() < 1) throw std::runtime_error("no gps data!");

  std::vector<uint64_t> imageTimes;
  readImageTimes(imageTimes);

  if(imageTimes.size() < 1) throw std::runtime_error("no camera time data!");

  uint64_t t_firstref = 0;
  if(gps)
  {
    t_firstref = gpsData[0].time;
  } else
  {
    t_firstref = odoData[0].time;
  }
  uint64_t t_ref = 0;
  size_t ref_index = 0;

  std::cout << "t_firstref=" << t_firstref << std::endl;

//  cv::namedWindow( "Window1" );// Create a window for display.
//  cv::namedWindow( "Window2" );// Create a window for display.
//  cv::namedWindow( "Window3" );// Create a window for display.
//  cv::namedWindow( "Window4" );// Create a window for display.


  for(size_t i = 0; i < imageTimes.size(); i++)
  {
    std::cout << "at t=" << imageTimes[i] << std::endl;
    std::stringstream ss;
    ss << std::setw(6) << std::setfill('0') << i;
    std::string n = ss.str();

    std::string filename1 = imageDir + "/cam0/img" + n + ".png";
    std::string filename2 = imageDir + "/cam1/img" + n + ".png";
    std::string filename3 = imageDir + "/cam2/img" + n + ".png";
    std::string filename4 = imageDir + "/cam3/img" + n + ".png";

    boost::filesystem::path imgFile(filename1);

    if(boost::filesystem::exists(imgFile))
    {

      std::cerr << "opening image " << filename1 << std::endl;
      std::cerr << "opening image " << filename2 << std::endl;
      std::cerr << "opening image " << filename3 << std::endl;
      std::cerr << "opening image " << filename4 << std::endl;

      std::vector<cv::Mat> images;

      cv::Mat image1 = cv::imread(filename1, CV_LOAD_IMAGE_GRAYSCALE);
      cv::Mat image2 = cv::imread(filename2, CV_LOAD_IMAGE_GRAYSCALE);
      cv::Mat image3 = cv::imread(filename3, CV_LOAD_IMAGE_GRAYSCALE);
      cv::Mat image4 = cv::imread(filename4, CV_LOAD_IMAGE_GRAYSCALE);

      images.push_back(image1);
      images.push_back(image2);
      images.push_back(image3);
      images.push_back(image4);

      uint64_t t_nextimages = imageTimes[i];

      if(t_nextimages < t_firstref) continue;

      while(!(t_ref > t_nextimages))
      {


        if(gps)
        {
          if(ref_index >= gpsData.size()) break;
          GpsData data = gpsData[ref_index++];
          _camOdoCal->addGpsIns(data.lat, data.lon, data.alt, data.roll, data.pitch, data.yaw, data.time);
          t_ref = data.time;
        } else
        {
          if(ref_index >= odoData.size()) break;
          OdometryData data = odoData[ref_index++];
          _camOdoCal->addOdometry(data.x, data.y, data.theta, data.time);
          t_ref = data.time;
        }
      }

      if(t_ref <= t_nextimages) throw std::runtime_error("timing error!");

      _camOdoCal->addFrameSet(images, t_nextimages);
    }
  }

  // run the calibration
  //_camOdoCal->run();
}

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
   bool gps;
   std::string input;

   //================= Handling Program options ==================
   boost::program_options::options_description desc("Allowed options");
   desc.add_options()
       ("help", "produce help message")
       ("calib,c", boost::program_options::value<std::string>(&calibDir)->default_value("calib"), "Directory containing camera calibration files.")
       ("input,i", boost::program_options::value<std::string>(&input)->required(), "Directory containing the input data (images, odometry, ...)")
       ("camera-count", boost::program_options::value<int>(&cameraCount)->default_value(1), "Number of cameras in rig.")
       ("f", boost::program_options::value<float>(&focal)->default_value(300.0f), "Nominal focal length.")
       ("output,o", boost::program_options::value<std::string>(&outputDir)->default_value("calibration_data"), "Directory to write calibration data to.")
       ("motions,m", boost::program_options::value<int>(&nMotions)->default_value(500), "Number of motions for calibration.")
       ("begin-stage", boost::program_options::value<int>(&beginStage)->default_value(0), "Stage to begin from.")
       ("preprocess", boost::program_options::bool_switch(&preprocessImages)->default_value(false), "Preprocess images.")
       ("optimize-intrinsics", boost::program_options::bool_switch(&optimizeIntrinsics)->default_value(false), "Optimize intrinsics in BA step.")
       ("data", boost::program_options::value<std::string>(&dataDir)->default_value("data"), "Location of folder which contains working data.")
       ("verbose,v", boost::program_options::bool_switch(&verbose)->default_value(false), "Verbose output")
       ("gps,g", boost::program_options::bool_switch(&gps)->default_value(false), "Use GPS data instead of odometry.")
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

   std::cout << "# INFO: Initializing... " << std::flush << std::endl;

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

   //===========================Initialize calibration==========================

   // read camera params
   std::vector<camodocal::CameraPtr> cameras(cameraCount);
   for (int i = 0; i < cameraCount; ++i)
   {
       boost::filesystem::path calibFilePath(calibDir);
       boost::filesystem::path maskFilePath(calibDir);

       std::ostringstream oss;
       oss << "camera_" << i << "_calib.yaml";
       calibFilePath /= oss.str();

       std::ostringstream mss;
       mss << "camera_" << i << "_mask.png";
       maskFilePath /= mss.str();

       camodocal::CameraPtr camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(calibFilePath.string());

       if(boost::filesystem::exists(maskFilePath))
       {
         std::cout << "using mask file " << maskFilePath.string() << std::endl;
         cv::Mat mask = cv::imread(maskFilePath.string(), CV_8UC1);
         camera->setMask(mask);
       }

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
   options.mode = CamRigOdoCalibration::OFFLINE;
   options.poseSource = GPS_INS;
   options.nMotions = nMotions;
   options.minKeyframeDistance = 0.2;
   options.minVOSegmentSize = 15;
   options.preprocessImages = preprocessImages;
   options.optimizeIntrinsics = optimizeIntrinsics;
   options.saveWorkingData = true;
   options.beginStage = beginStage;
   options.dataDir = dataDir;
   options.verbose = verbose;

   CamRigOdoCalibration camRigOdoCalib(cameras, options);
   camRigOdoCalib.cameraSystem().setReferenceCamera(0);

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
   // camRigOdoCalib.addGpsIns(lat, lon, alt, roll, pitch, yaw, timestamp);
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

   DataFeeder df(&camRigOdoCalib, input);
   boost::thread dataFeeder( boost::bind(&DataFeeder::addData, &df, gps));

   camRigOdoCalib.start();
   dataFeeder.join();

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
