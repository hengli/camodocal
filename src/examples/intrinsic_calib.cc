#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camodocal/CataCameraCalibration.h"
#include "camodocal/Chessboard.h"

int main(int argc, char** argv)
{
    cv::Size boardSize;
    float squareSize;
    std::string inputDir;
    std::string cameraName;
    std::string prefix;
    std::string fileExtension;
    bool useOpenCV;
    bool verbose;

    //========= Handling Program options =========
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("width,w", boost::program_options::value<int>(&boardSize.width)->default_value(9), "Number of inner corners on the chessboard pattern in x direction")
        ("height,h", boost::program_options::value<int>(&boardSize.height)->default_value(6), "Number of inner corners on the chessboard pattern in y direction")
        ("size,s", boost::program_options::value<float>(&squareSize)->default_value(120.f), "Size of one square in mm")
        ("input,i", boost::program_options::value<std::string>(&inputDir)->default_value("images"), "Input directory containing chessboard images")
        ("prefix,p", boost::program_options::value<std::string>(&prefix)->default_value("image"), "Prefix of images")
        ("file-extension,e", boost::program_options::value<std::string>(&fileExtension)->default_value(".bmp"), "File extension of images")
        ("camera-name", boost::program_options::value<std::string>(&cameraName)->default_value("camera"), "Name of camera")
        ("opencv", boost::program_options::bool_switch(&useOpenCV)->default_value(false), "Use OpenCV to detect corners")
        ("verbose,v", boost::program_options::bool_switch(&verbose)->default_value(false), "Verbose output")
        ;

    boost::program_options::positional_options_description pdesc;
    pdesc.add("input", 1);

    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::command_line_parser(argc, argv).options(desc).positional(pdesc).run(), vm);
    boost::program_options::notify(vm);

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return 1;
    }

    if (!boost::filesystem::exists(inputDir) && !boost::filesystem::is_directory(inputDir))
    {
        std::cerr << "# ERROR: Cannot find input directory " << inputDir << "." << std::endl;
        return 1;
    }

    // look for images in input directory
    std::vector<std::string> images;
    boost::filesystem::directory_iterator itr;
    for (boost::filesystem::directory_iterator itr(inputDir); itr != boost::filesystem::directory_iterator(); ++itr)
    {
        if (!boost::filesystem::is_regular_file(itr->status()))
        {
            continue;
        }

        std::string filename = itr->path().filename().string();

        // check if prefix matches
        if (!prefix.empty())
        {
            if (filename.compare(0, prefix.length(), prefix) != 0)
            {
                continue;
            }
        }

        // check if file extension matches
        if (filename.compare(filename.length() - fileExtension.length(), fileExtension.length(), fileExtension) != 0)
        {
            continue;
        }

        images.push_back(itr->path().string());

        if (verbose)
        {
            std::cerr << "# INFO: Adding " << images.back() << std::endl;
        }
    }

    if (images.empty())
    {
        std::cerr << "# ERROR: No chessboard images found." << std::endl;
        return 1;
    }

    if (verbose)
    {
        std::cerr << "# INFO: # images: " << images.size() << std::endl;
    }

    cv::Mat image = cv::imread(images.front(), -1);
    const cv::Size frameSize = image.size();

    camodocal::CataCameraCalibration calibration(cameraName, frameSize);
    calibration.setVerbose(verbose);

    for (size_t i = 0; i < images.size(); ++i)
    {
        image = cv::imread(images.at(i), -1);

        camodocal::Chessboard chessboard(boardSize, image);

        chessboard.findCorners(useOpenCV);
        if (chessboard.cornersFound())
        {
            if (verbose)
            {
                std::cerr << "# INFO: Detected chessboard in image " << i + 1 << std::endl;
            }

            calibration.addChessboardData(chessboard.getCorners());

            cv::Mat sketch; 
            chessboard.getSketch().copyTo(sketch); 

            cv::imshow("Image", sketch);
            cv::waitKey(50);
        }
        else if (verbose)
        {
            std::cerr << "# INFO: Did not detect chessboard in image " << i + 1 << std::endl;
        }
    }

    if (calibration.sampleCount() < 10)
    {
        std::cerr << "# ERROR: Insufficient number of detected chessboards." << std::endl;
        return 1;
    }

    if (verbose)
    {
        std::cerr << "# INFO: Calibrating..." << std::endl;
    }

    calibration.calibrate(boardSize, squareSize);
    calibration.writeParams(cameraName + "_camera_calib.yaml");

    if (verbose)
    {
        std::cerr << "# INFO: Wrote calibration file to " << cameraName + "_camera_calib.yaml" << std::endl;
    }

    return 0;
}
