// main.cpp
// Main executable source for the streamer program

#include <iostream>
#include <string>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <boost/program_options.hpp>

#include "KITTIVisionParser.hpp"
#include "cv_networking/message/StereoStreamMessages.hpp"
#include "cv_networking/client/StereoStreamerClient.hpp"

enum CmdParseResult {
    ARGS_HELP, ARGS_VALID, ARGS_INVALID
};

#define DEFAULT_SEVER_PORT 7000

// Get command line arguments that are required
CmdParseResult GetCmdArgs(int argc, char** argv, std::string& calibFolder, std::string& dataFolder, std::string& serverAddress, int& serverPort, bool& isRectifiedData);

// Convert dataset calib to cv networking calib message
CVNetwork::Message::StereoCalibMessage ConvertToCalibMessage(const Calib& calib);

// Create stereo message from data set sample
CVNetwork::Message::StereoMessage GetSteroMessageFromSample(const DataSample& sample);

int main(int argc, char** argv)
{
    std::string calibFolder, dataFolder, serverAddress;
    int port = DEFAULT_SEVER_PORT;
    bool isRectifiedData = false;

    // get required arguments
    CmdParseResult parseResult = GetCmdArgs(argc, argv, calibFolder, dataFolder, serverAddress, port, isRectifiedData);

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

    // parse calib and data from KITTI dataset
    KITTIVisionParser parser(calibFolder, dataFolder, isRectifiedData);
    Calib calib = parser.GetParsedCalibData();

    std::vector<DataSample> samples;
    parser.GetParsedDataSamples(samples);

    // convert calib to networking calib record
    CVNetwork::Message::StereoCalibMessage calibMessage = ConvertToCalibMessage(calib);

    // connect to server
    CVNetwork::Clients::StereoStreamerClient client(calibMessage);
    if (client.ConnectToReconstructServer(serverAddress, port))
    {
        std::cout << "\nConnected to reconstruction server" << std::endl;

        // run client to begin stereo stream
        client.Run();

        // Add each sample to the client's streaming queue
        for (const DataSample& sample : samples) {
            std::cout << "\nSending sample to server: " << sample.ID;
            client.AddStereoDataToQueue(GetSteroMessageFromSample(sample));
        }
    }
    else {
        std::cout << "\nUnable to connect to server at " << serverAddress << ": " << port;
        return -1;
    }

    std::cout << "\nMain thread has added all samples to queue. Ctrl-C to end the program" << std::endl;
    while (true) {};

    std::cout << std::endl;
    return 0;
}

CmdParseResult GetCmdArgs(int argc, char** argv, std::string& calibFolder, std::string& dataFolder, std::string& serverAddress, int& serverPort, bool& isRectifiedData)
{
    boost::program_options::options_description desc("Options");
    desc.add_options()
            ("help,h", "View help message")
            ("calib_folder", boost::program_options::value<std::string>()->required(), "The path to the folder with the calibration text files")
            ("data_folder", boost::program_options::value<std::string>()->required(), "The path to the folder with the image files. This should contain the image_002, image_003, and oxts folders")
            ("server_ip", boost::program_options::value<std::string>()->required(), "The IPv4 address of the reconstruction server")
            ("server_port", boost::program_options::value<int>(), "The port on the server for the reconstruction application")
            ("rectified", "Rectified images being streamed. Send rectified calibration");

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
            std::cout << "--server_port: The port for the reconstruction application on the server (default = 7000)";
            std::cout << "--rectified: Set arg if rectified images are being streamed\n";
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

                // optional server port
                if (vm.count("server_port")) {
                    serverPort = vm["server_port"].as<int>();
                }
                else {
                    serverPort = DEFAULT_SEVER_PORT;
                }

                // optional is rectified data being streamed
                if (vm.count("rectified")) {
                    isRectifiedData = true;
                }
                else {
                    isRectifiedData = false;
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

// Convert to calib message
CVNetwork::Message::StereoCalibMessage ConvertToCalibMessage(const Calib& calib)
{
    CVNetwork::Message::StereoCalibMessage message{};

    message.fx1 = calib.K1(0, 0);
    message.fy1 = calib.K1(1, 1);
    message.cx1 = calib.K1(0, 2);
    message.cy1 = calib.K1(1, 2);

    message.d11 = calib.D1(0);
    message.d12 = calib.D1(1);
    message.d13 = calib.D1(2);
    message.d14 = calib.D1(3);
    message.d15 = calib.D1(4);
    message.d16 = 0;
    message.d17 = 0;
    message.d18 = 0;

    message.fx2 = calib.K2(0, 0);
    message.fy2 = calib.K2(1, 1);
    message.cx2 = calib.K2(0, 2);
    message.cy2 = calib.K2(1, 2);

    message.d21 = calib.D2(0);
    message.d22 = calib.D2(1);
    message.d23 = calib.D2(2);
    message.d24 = calib.D2(3);
    message.d25 = calib.D2(4);
    message.d26 = 0;
    message.d27 = 0;
    message.d28 = 0;

    message.t1 = calib.T(0);
    message.t2 = calib.T(1);
    message.t3 = calib.T(2);

    message.r1 = calib.R(0, 0);
    message.r2 = calib.R(0, 1);
    message.r3 = calib.R(0, 2);
    message.r4 = calib.R(1, 0);
    message.r5 = calib.R(1, 1);
    message.r6 = calib.R(1, 2);
    message.r7 = calib.R(2, 0);
    message.r8 = calib.R(2, 1);
    message.r9 = calib.R(2, 2);

    return message;
}

// Create stereo message from data set sample
CVNetwork::Message::StereoMessage GetSteroMessageFromSample(const DataSample& sample)
{
    CVNetwork::Message::StereoMessage message;

    // convert images to png encoded bytes
    cv::Mat leftImage = cv::imread(sample.Camera1ImagePath, cv::IMREAD_COLOR);
    cv::imencode(".png", leftImage, message.LeftImageData);

    cv::Mat rightImage = cv::imread(sample.Camera2ImagePath, cv::IMREAD_COLOR);
    cv::imencode(".png", rightImage, message.RightImageData);

    // pose data
    message.X = sample.T[0];
    message.Y = sample.T[1];
    message.Z = sample.T[2];

    message.R1 = sample.R(0, 0);
    message.R2 = sample.R(0, 1);
    message.R3 = sample.R(0, 2);
    message.R4 = sample.R(1, 0);
    message.R5 = sample.R(1, 1);
    message.R6 = sample.R(1, 2);
    message.R7 = sample.R(2, 0);
    message.R8 = sample.R(2, 1);
    message.R9 = sample.R(2, 2);

    return message;
}