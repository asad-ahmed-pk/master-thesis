//
// test_server.cpp
// Executable for testing the server functionality of the library
//

#include <iostream>
#include <thread>
#include <chrono>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>

#include "server/ReconstructionServer.hpp"

int main(int argc, char** argv)
{
    // create server and start
    CVNetwork::Servers::ReconstructionServer server;
    server.StartServer();

    // main thread sleeps until server thread has fetched some data
    std::cout << "\nMain thread waiting for 5 seconds before checking stereo queue" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // check queue for stereo data
    CVNetwork::Message::StereoMessage message;
    if (server.GetNextStereoDataFromQueue(message))
    {
        std::cout << "\nGot stereo item from queue:\n";
        std::cout << "\nX, Y, Z = " << message.X << ", " << message.Y << ", " << message.Z << std::endl;

        // read opencv mats and write to disk
        cv::Mat img1, img2;
        cv::imdecode(message.LeftImageData, cv::IMREAD_COLOR, &img1);
        cv::imdecode(message.RightImageData, cv::IMREAD_COLOR, &img2);

        if (img1.data && img2.data) {
            std::cout << "\nSaved images that were received from the client to disk" << std::endl;
            cv::imwrite("img1_from_client.png", img1);
            cv::imwrite("img2_from_client.png", img2);
        }
    }

    std::cout << "\n\nPress ENTER to stop server" << std::endl;
    std::cin.get();

    return 0;
}
