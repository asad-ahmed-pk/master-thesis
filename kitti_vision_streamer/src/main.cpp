// main.cpp
// Main executable source for the streamer program

#include <iostream>
#include <string>
#include <boost/program_options.hpp>

#include "KITTIVisionParser.hpp"

enum CmdParseResult {
    ARGS_HELP, ARGS_VALID, ARGS_INVALID
};

#define DEFAULT_SEVER_PORT 7000

// Get command line arguments that are required
CmdParseResult GetCmdArgs(int argc, char** argv, std::string& calibFolder, std::string& dataFolder, std::string& serverAddress, int& serverPort);

int main(int argc, char** argv)
{
    std::string calibFolder, dataFolder, serverAddress;
    int port = DEFAULT_SEVER_PORT;

    // get required arguments
    CmdParseResult parseResult = GetCmdArgs(argc, argv, calibFolder, dataFolder, serverAddress, port);

    switch (parseResult)
    {
        case CmdParseResult::ARGS_INVALID:
            std::cerr << "\nInvalid program arguments";
            std::cout << "\nUse --help or -h for help" << std::endl;
            return 1;

        case CmdParseResult::ARGS_HELP:
            return 0;

        default:
            break;
    }

    // all required params parsed
    KITTIVisionParser parser(calibFolder, dataFolder);
    Calib calib = parser.GetParsedCalibData();

    std::cout << "\n\nParsed calib data..." << std::endl;
    std::cout << "K1:\n" << calib.K1;
    std::cout << "\nK2:\n" << calib.K2;

    std::cout << "\nD1:\n" << calib.D1;
    std::cout << "\nD2:\n" << calib.D2;

    std::cout << "\nT:\n" << calib.T;
    std::cout << "\nR:\n" << calib.R;

    // get data samples
    std::vector<DataSample> samples;
    parser.GetParsedDataSamples(samples);

    // print first 2
    std::cout << "\nFirst 2 samples" << std::endl;
    for (int i = 0; i < 2; i++)
    {
        std::cout << "\n\n" << i << ": \n";
        std::cout << "\nImage 1: " << samples[i].Camera1ImagePath;
        std::cout << "\nImage 2: " << samples[i].Camera2ImagePath;
        std::cout << "\n(X, Y, Z): " << "(" << samples[i].T[0] << ", " << samples[i].T[1] << ", " << samples[i].T[2] << ")";
        std::cout << "\nR:\n" << samples[i].R;
    }

    std::cout << std::endl;
    return 0;
}

CmdParseResult GetCmdArgs(int argc, char** argv, std::string& calibFolder, std::string& dataFolder, std::string& serverAddress, int& serverPort)
{
    boost::program_options::options_description desc("Options");
    desc.add_options()
            ("help,h", "View help message")
            ("calib_folder", boost::program_options::value<std::string>()->required(), "The path to the folder with the calibration text files")
            ("data_folder", boost::program_options::value<std::string>()->required(), "The path to the folder with the image files. This should contain the image_002, image_003, and oxts folders")
            ("server_ip", boost::program_options::value<std::string>()->required(), "The IPv4 address of the reconstruction server")
            ("server_port", boost::program_options::value<int>(), "The port on the server for the reconstruction application");

    boost::program_options::variables_map vm;
    try
    {
        boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);

        // help option
        if (vm.count("help"))
        {
            std::cout << "\nKITTI Vision Dataset Streamer";
            std::cout << "\n\nRequired arguments:\n";
            std::cout << "--calib_folder: The path to the folder with the calibration text files";
            std::cout << "\n--data_folder: The path to the folder with the image files. This should contain the image_002, image_003, and oxts folders";
            std::cout << "\n--server_ip: The IPv4 address of the reconstruction server";
            std::cout << "\n\nOptional arguments:\n";
            std::cout << "--server_port: The port for the reconstruction application on the server (default = 7000)\n";
            std::cout << std::endl;

            return CmdParseResult::ARGS_HELP;
        }
        else
        {
            if (vm.count("calib_folder") && vm.count("data_folder") && vm.count("server_ip"))
            {
                // save options from cmd line
                calibFolder = vm["calib_folder"].as<std::string>();
                dataFolder = vm["data_folder"].as<std::string>();
                serverAddress = vm["server_ip"].as<std::string>();

                if (vm.count("server_port")) {
                    serverPort = vm["server_port"].as<int>();
                }
                else {
                    serverPort = DEFAULT_SEVER_PORT;
                }

                return CmdParseResult::ARGS_VALID;
            }
            else {
                vm.notify();
                return CmdParseResult::ARGS_INVALID;
            }
        }
    }
    catch (boost::program_options::error& e) {
        return CmdParseResult::ARGS_INVALID;
    }
}
