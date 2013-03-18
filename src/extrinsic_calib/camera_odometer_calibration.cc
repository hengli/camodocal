#include <boost/program_options.hpp>
#include <iostream>

#include "camodocal/CameraOdometerCalibration.h"

int
main(int argc, char** argv)
{
    std::string inputFile;
    bool verbose;

    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("input,i", boost::program_options::value<std::string>(&inputFile)->default_value("odo_cam.txt"), "Input file containing motion data")
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

    camodocal::CameraOdometerCalibration cal;
    if (!cal.readMotionSegmentsFromFile(inputFile))
    {
        std::cout << "# ERROR: Unable to read motion segments from " << inputFile << std::endl;

        return 1;
    }

    cal.setVerbose(verbose);

    Eigen::Matrix4d H_cam_odo;
    if (!cal.calibrate(H_cam_odo))
    {
        std::cout << "# ERROR: Calibration failed." << std::endl;

        return 1;
    }

    std::cout << "# INFO: H_cam_odo:" << std::endl << H_cam_odo << std::endl;

    return 0;
}
