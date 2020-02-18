//
// ReconstructionServer.hpp
// The reconstruction server that reads from the stereo stream and continuously reconstructs the scene
// Connects to the visualisation module as a client and streams the reconstructed point clouds
//

#include "server/ReconstructionServer.hpp"
#include "config/ConfigParser.hpp"
#include "camera/CameraCalibParser.hpp"
#include "camera/CameraCompute.hpp"
#include "server/MessageConverter.hpp"
#include "cv_networking/message/StereoStreamMessages.hpp"
#include "server/server_constants.hpp"

#include <memory>
#include <thread>
#include <chrono>
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>
#include <opencv2/highgui/highgui.hpp>

namespace Server
{
#define CALIB_FILE_PATH "calib.json"

    // Constructor
    ReconstructionServer::ReconstructionServer()
    {
        // parse config
        Config::ConfigParser parser;
        m_Config = parser.ParseConfig();

        // check for calib file
        if (boost::filesystem::exists(CALIB_FILE_PATH))
        {
            Camera::CameraCalibParser calibParser;
            m_Calib = std::make_unique<Camera::Calib::StereoCalib>();
            calibParser.ParseStereoCalibJSONFile(CALIB_FILE_PATH, *m_Calib);
        }

        // create the networking server instance from cv_networking lib
        m_ReconstructionServer = std::make_unique<CVNetwork::Servers::ReconstructionServer>(m_Config.Server.ServerPort, m_Calib == nullptr);
    }

    // Destructor
    ReconstructionServer::~ReconstructionServer()
    {

    }

    // Run server
    ReconstructServerStatusCode ReconstructionServer::Run()
    {
        // start processing thread
        m_ProcessingThread = std::thread(&ReconstructionServer::RunProcessingThread, this);

        // wait until processing begins
        while (!m_ProcessingStarted) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }

        // print message that client connected and is streaming
        m_UserInterface.PrintClientConnectedMessage();

        // main event loop: get user prompt and process user's command
        do
        {
            UserOption option = m_UserInterface.GetUserCommandOption();

            switch (option) {
                case USER_OPTION_PRINT_STATUS:
                    // TODO: get number of messages in queue and send as param
                    m_UserInterface.PrintStatusMessage(m_NumFramesProcessed, 0);
                    break;

                case USER_OPTION_QUIT:
                    // set the exit flag
                    m_UserRequestedToQuit = true;
                    break;

                default:
                    break;
            }
        } while (!m_UserRequestedToQuit);

        // if user requested to quit, wait for the processing thread
        m_ProcessingThread.join();
        m_ReconstructionServer->StopServer();

        // TODO: handle the exit code better: user can also quit
        return ReconstructServerStatusCode::SERVER_CLIENT_DISCONNECTED;
    }

    // Process thread run
    void ReconstructionServer::RunProcessingThread()
    {
        m_ReconstructionServer->StartServer();
        bool isRunning = true;

        // wait for calib if required from client
        if (m_Calib == nullptr)
        {
            while (m_Calib == nullptr)
            {
                if (m_ReconstructionServer->IsCalibAvailable()) {
                    CVNetwork::Message::StereoCalibMessage calibMessage = m_ReconstructionServer->GetCalibMessage();
                    m_Calib = std::make_unique<Camera::Calib::StereoCalib>(Utility::MessageConverter::CovertCalibMessage(calibMessage));
                    break;
                }
                else {
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
            }
        }

        // calib setup by this point - perform stereo rectification
        Camera::Calib::StereoCalib stereoSetup;
        Camera::CameraCompute cameraCompute(*m_Calib);
        stereoSetup = std::move(cameraCompute.GetRectifiedStereoSettings());

        // construct the pipeline with the calib information
        m_ReconstructionPipeline = std::make_unique<Pipeline::ReconstructionPipeline>(m_Config, stereoSetup, !m_Config.Reconstruction.ShouldRectifyImages);

        // calib data will be loaded by now - expecting a constant stream of stereo messages at this point
        // these are in the server's data queue
        CVNetwork::Message::StereoMessage message;
        Pipeline::StereoFrame frame;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr finalPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>());

        m_ProcessingStarted = true;

        while (isRunning && !m_UserRequestedToQuit)
        {
            if (m_ReconstructionServer->GetNextStereoDataFromQueue(message))
            {
                //std::cout << "\nProcessing frame #" << n << " in queue" << std::endl;

                // got a stereo message from the client process with 3D reconstruct
                frame = std::move(Utility::MessageConverter::ConvertStereoMessage(message));
                frame.ID = m_NumFramesProcessed;

                // clear all points from temp point cloud
                pointCloud->clear();

                // process this frame
                m_ReconstructionPipeline->ProcessFrame(frame, pointCloud);

                // append all points from this point cloud to final point cloud
                for (const auto& p : *pointCloud) {
                    finalPointCloud->push_back(p);
                }
                m_NumFramesProcessed++;
            }
        }

        // save point cloud to file for now
        // TODO: rename point cloud to a better file name (timestamped maybe?)
        std::cout << "\nServer finished receiving stream. Saving point cloud to file" << std::endl;
        pcl::io::savePCDFileBinary("final_point_cloud.pcd", *finalPointCloud);
    }
}
