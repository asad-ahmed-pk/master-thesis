//
// test_stereo_client.cpp
// Test program for testing the stereo client connecting to the reconstruction server test program
//

#include <iostream>
#include <string>
#include <opencv2/imgcodecs/imgcodecs.hpp>

#include "message/StereoStreamMessages.hpp"
#include "client/StereoStreamerClient.hpp"

int main(int argc, char** argv)
{
    cv::Mat img1, img2;

    // optional command line arguments to 2 images
    if (argc >= 3)
    {
        std::string img1Path = argv[1];
        std::string img2Path = argv[2];

        img1 = cv::imread(img1Path, cv::IMREAD_COLOR);
        img2 = cv::imread(img2Path, cv::IMREAD_COLOR);
    }

    // create test calib message
    CVNetwork::Message::StereoCalibMessage calibMessage{};
    calibMessage.fx1 = 100;
    calibMessage.fx2 = 200;

    // connect to reconstruction server
    CVNetwork::Clients::StereoStreamerClient client(calibMessage);

    if (client.ConnectToReconstructServer("127.0.0.1", 7000))
    {
        std::cout << "\nConnected to reconstruction server on port 7000" << std::endl;

        // initiate stereo flow
        client.Run();

        // if 2 stereo images were loaded - send them to the server
        if (img1.data && img2.data)
        {
            CVNetwork::Message::StereoMessage message{};

            // encode images
            bool encodedImage1 = cv::imencode(".png", img1, message.LeftImageData);
            bool encodedImage2 = cv::imencode(".png", img2, message.RightImageData);

            if (encodedImage1 && encodedImage2)
            {
                message.X = 100;
                message.Y = 200;
                message.Z = 300;

                std::cout << "\nSending stereo image to server..." << std::endl;
                client.AddStereoDataToQueue(message);
            }
        }

        // input before closing test program
        std::cout << "\n\nPress ENTER to exit client" << std::endl;
        std::cin.get();
    }

    return 0;
}

