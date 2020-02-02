//
// test_stereo_client.cpp
// Test program for testing the stereo client connecting to the reconstruction server test program
//

#include <iostream>

#include "message/StereoStreamMessages.hpp"
#include "client/StereoStreamerClient.hpp"

int main(int argc, char** argv)
{
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

        std::cin.get();
    }

    return 0;
}

