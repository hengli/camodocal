#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <iomanip>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camodocal/chessboard/Chessboard.h"
#include "camodocal/calib/StereoCameraCalibration.h"
#include "../gpl/gpl.h"

int main(int argc, char** argv)
{
    cv::Size boardSize;
    float squareSize;
    std::string inputDir;
    std::string outputDir;
    std::string cameraModel;
    std::string cameraNameL, cameraNameR;
    std::string prefixL, prefixR;
    std::string fileExtension;
    bool useOpenCV;
    bool viewResults;
    bool verbose;

    //========= Handling Program options =========
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("width,w", boost::program_options::value<int>(&boardSize.width)->default_value(9), "Number of inner corners on the chessboard pattern in x direction")
        ("height,h", boost::program_options::value<int>(&boardSize.height)->default_value(6), "Number of inner corners on the chessboard pattern in y direction")
        ("size,s", boost::program_options::value<float>(&squareSize)->default_value(120.f), "Size of one square in mm")
        ("input,i", boost::program_options::value<std::string>(&inputDir)->default_value("images"), "Input directory containing chessboard images")
        ("output,o", boost::program_options::value<std::string>(&outputDir)->default_value("."), "Output directory containing calibration data")
        ("prefix-l", boost::program_options::value<std::string>(&prefixL)->default_value("left"), "Prefix of images from left camera")
        ("prefix-r", boost::program_options::value<std::string>(&prefixR)->default_value("right"), "Prefix of images from right camera")
        ("file-extension,e", boost::program_options::value<std::string>(&fileExtension)->default_value(".bmp"), "File extension of images")
        ("camera-model", boost::program_options::value<std::string>(&cameraModel)->default_value("mei"), "Camera model: kannala-brandt | mei | pinhole")
        ("camera-name-l", boost::program_options::value<std::string>(&cameraNameL)->default_value("camera_left"), "Name of left camera")
        ("camera-name-r", boost::program_options::value<std::string>(&cameraNameR)->default_value("camera_right"), "Name of right camera")
        ("opencv", boost::program_options::bool_switch(&useOpenCV)->default_value(false), "Use OpenCV to detect corners")
        ("view-results", boost::program_options::bool_switch(&viewResults)->default_value(false), "View results")
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

    camodocal::Camera::ModelType modelType;
    if (boost::iequals(cameraModel, "kannala-brandt"))
    {
        modelType = camodocal::Camera::KANNALA_BRANDT;
    }
    else if (boost::iequals(cameraModel, "mei"))
    {
        modelType = camodocal::Camera::MEI;
    }
    else if (boost::iequals(cameraModel, "pinhole"))
    {
        modelType = camodocal::Camera::PINHOLE;
    }
    else if (boost::iequals(cameraModel, "scaramuzza"))
    {
        modelType = camodocal::Camera::SCARAMUZZA;
    }
    else
    {
        std::cerr << "# ERROR: Unknown camera model: " << cameraModel << std::endl;
        return 1;
    }

    switch (modelType)
    {
    case camodocal::Camera::KANNALA_BRANDT:
        std::cout << "# INFO: Camera model: Kannala-Brandt" << std::endl;
        break;
    case camodocal::Camera::MEI:
        std::cout << "# INFO: Camera model: Mei" << std::endl;
        break;
    case camodocal::Camera::PINHOLE:
        std::cout << "# INFO: Camera model: Pinhole" << std::endl;
        break;
    case camodocal::Camera::SCARAMUZZA:
        std::cout << "# INFO: Camera model: Scaramuzza-Omnidirect" << std::endl;
        break;
    }

    // look for images in input directory
    std::vector<std::string> imageFilenamesL, imageFilenamesR;
    boost::filesystem::directory_iterator itr;
    for (boost::filesystem::directory_iterator itr(inputDir); itr != boost::filesystem::directory_iterator(); ++itr)
    {
        if (!boost::filesystem::is_regular_file(itr->status()))
        {
            continue;
        }

        std::string filename = itr->path().filename().string();

        // check if file extension matches
        if (filename.compare(filename.length() - fileExtension.length(), fileExtension.length(), fileExtension) != 0)
        {
            continue;
        }

        // check if prefix matches
        if (prefixL.empty() || (!prefixL.empty() && (filename.compare(0, prefixL.length(), prefixL) == 0)))
        {
            imageFilenamesL.push_back(itr->path().string());

            if (verbose)
            {
                std::cerr << "# INFO: Adding " << imageFilenamesL.back() << std::endl;
            }
        }
        if (prefixR.empty() || (!prefixR.empty() && (filename.compare(0, prefixR.length(), prefixR) == 0)))
        {
            imageFilenamesR.push_back(itr->path().string());

            if (verbose)
            {
                std::cerr << "# INFO: Adding " << imageFilenamesR.back() << std::endl;
            }
        }
    }

    if (imageFilenamesL.empty() || imageFilenamesR.empty())
    {
        std::cerr << "# ERROR: No chessboard images found." << std::endl;
        return 1;
    }

    if (imageFilenamesL.size() != imageFilenamesR.size())
    {
        std::cerr << "# ERROR: # chessboard images from left and right cameras do not match." << std::endl;
        return 1;
    }

    bool matchImages = true;
    std::sort(imageFilenamesL.begin(), imageFilenamesL.end());
    std::sort(imageFilenamesR.begin(), imageFilenamesR.end());

    for (size_t i = 0; i < imageFilenamesL.size(); ++i)
    {
        std::string filenameL = boost::filesystem::path(imageFilenamesL.at(i)).filename().string();
        std::string filenameR = boost::filesystem::path(imageFilenamesR.at(i)).filename().string();

        if (filenameL.compare(prefixL.length(),
                              filenameL.size() - prefixL.length(),
                              filenameR,
                              prefixR.length(),
                              filenameR.size() - prefixR.length()) != 0)
        {
            matchImages = false;

            if (verbose)
            {
                std::cerr << "# ERROR: Filenames do not match: "
                          << imageFilenamesL.at(i) << " " << imageFilenamesR.at(i) << std::endl;
            }
        }
    }

    if (!matchImages)
    {
        return 1;
    }

    if (verbose)
    {
        std::cerr << "# INFO: # images: " << imageFilenamesL.size() << std::endl;
    }

    cv::Mat imageL = cv::imread(imageFilenamesL.front(), -1);
    cv::Mat imageR;
    const cv::Size frameSize = imageL.size();

    camodocal::StereoCameraCalibration calibration(modelType, cameraNameL, cameraNameR, frameSize, boardSize, squareSize);
    calibration.setVerbose(verbose);

    std::vector<bool> chessboardFoundL(imageFilenamesL.size(), false);
    std::vector<bool> chessboardFoundR(imageFilenamesR.size(), false);
    for (size_t i = 0; i < imageFilenamesL.size(); ++i)
    {
        imageL = cv::imread(imageFilenamesL.at(i), -1);
        imageR = cv::imread(imageFilenamesR.at(i), -1);

        camodocal::Chessboard chessboardL(boardSize, imageL);
        camodocal::Chessboard chessboardR(boardSize, imageR);

        chessboardL.findCorners(useOpenCV);
        chessboardR.findCorners(useOpenCV);
        if (chessboardL.cornersFound() && chessboardR.cornersFound())
        {
            if (verbose)
            {
                std::cerr << "# INFO: Detected chessboard in image " << i + 1 << std::endl;
            }

            calibration.addChessboardData(chessboardL.getCorners(),
                                          chessboardR.getCorners());

            cv::Mat sketch;
            chessboardL.getSketch().copyTo(sketch);

            cv::imshow("Image - Left", sketch);

            chessboardR.getSketch().copyTo(sketch);

            cv::imshow("Image - Right", sketch);

            cv::waitKey(50);
        }
        else if (verbose)
        {
            std::cerr << "# INFO: Did not detect chessboard in image " << i + 1 << std::endl;
        }
        chessboardFoundL.at(i) = chessboardL.cornersFound();
        chessboardFoundR.at(i) = chessboardR.cornersFound();
    }
    cv::destroyWindow("Image - Left");
    cv::destroyWindow("Image - Right");

    if (calibration.sampleCount() < 10)
    {
        std::cerr << "# ERROR: Insufficient number of detected chessboards." << std::endl;
        return 1;
    }

    if (verbose)
    {
        std::cerr << "# INFO: Calibrating..." << std::endl;
    }

    double startTime = camodocal::timeInSeconds();

    calibration.calibrate();
    calibration.writeParams(outputDir);

    if (verbose)
    {
        std::cout << "# INFO: Calibration took a total time of "
                  << std::fixed << std::setprecision(3) << camodocal::timeInSeconds() - startTime
                  << " sec.\n";
    }

    if (verbose)
    {
        std::cerr << "# INFO: Wrote calibration files to " << boost::filesystem::absolute(outputDir).string() << std::endl;
    }

    if (viewResults)
    {
        std::vector<cv::Mat> cbImagesL, cbImagesR;
        std::vector<std::string> cbImageFilenamesL;
        std::vector<std::string> cbImageFilenamesR;

        for (size_t i = 0; i < imageFilenamesL.size(); ++i)
        {
            if (!chessboardFoundL.at(i) || !chessboardFoundR.at(i))
            {
                continue;
            }

            cbImagesL.push_back(cv::imread(imageFilenamesL.at(i), -1));
            cbImageFilenamesL.push_back(imageFilenamesL.at(i));

            cbImagesR.push_back(cv::imread(imageFilenamesR.at(i), -1));
            cbImageFilenamesR.push_back(imageFilenamesR.at(i));
        }

        // visualize observed and reprojected points
        calibration.drawResults(cbImagesL, cbImagesR);

        for (size_t i = 0; i < cbImagesL.size(); ++i)
        {
            cv::putText(cbImagesL.at(i), cbImageFilenamesL.at(i), cv::Point(10,20),
                        cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 255),
                        1, CV_AA);
            cv::imshow("Image - Left", cbImagesL.at(i));
            cv::putText(cbImagesR.at(i), cbImageFilenamesR.at(i), cv::Point(10,20),
                        cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 255),
                        1, CV_AA);
            cv::imshow("Image - Right", cbImagesR.at(i));
            cv::waitKey(0);
        }
    }

    return 0;
}
