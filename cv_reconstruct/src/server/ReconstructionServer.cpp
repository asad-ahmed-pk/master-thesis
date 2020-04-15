//
// ReconstructionServer.hpp
// The reconstruction server that reads from the stereo stream and continuously reconstructs the scene
// Connects to the visualisation module as a client and streams the reconstructed point clouds
//

#include "config/ConfigParser.hpp"
#include "camera/CameraCalibParser.hpp"
#include "camera/CameraCompute.hpp"
#include "cv_networking/message/StereoStreamMessages.hpp"
#include "server/MessageConverter.hpp"
#include "server/server_constants.hpp"
#include "server/ReconstructionServer.hpp"

#include <memory>
#include <thread>
#include <chrono>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define CALIB_FILE_PATH "calib.json"
#define IMAGE_DOWNSIZE_FACTOR 0.3

namespace Server
{
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

        // notify user that server has started
        std::cout << "\nServer started. Waiting for client to connect." << std::endl;

        // wait until processing begins
        while (!m_ProcessingStarted) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }

        // print message that client connected and is streaming
        std::cout << "\nClient connected. Recieving stereo stream..." << std::endl;
        
        // create and run visualiser (on main thread)
        m_Visualiser = std::make_unique<Visualisation::Visualiser>(m_ReconstructionSystem->GetMapDataBase());
        m_Visualiser->Run();
        
        // if here - window was closed
        m_UserRequestedToQuit = true;

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
        stereoSetup = cameraCompute.GetRectifiedStereoSettings();
        
        // adjust intrinsics since downscaling images
        stereoSetup.LeftCameraCalib.K(0, 0) = stereoSetup.LeftCameraCalib.K(0, 0) * IMAGE_DOWNSIZE_FACTOR;
        stereoSetup.LeftCameraCalib.K(1, 1) = stereoSetup.LeftCameraCalib.K(1, 1) * IMAGE_DOWNSIZE_FACTOR;
        stereoSetup.LeftCameraCalib.K(0, 2) = stereoSetup.LeftCameraCalib.K(0, 2) * IMAGE_DOWNSIZE_FACTOR;
        stereoSetup.LeftCameraCalib.K(1, 2) = stereoSetup.LeftCameraCalib.K(1, 2) * IMAGE_DOWNSIZE_FACTOR;
        
        stereoSetup.RightCameraCalib.K(0, 0) = stereoSetup.RightCameraCalib.K(0, 0) * IMAGE_DOWNSIZE_FACTOR;
        stereoSetup.RightCameraCalib.K(1, 1) = stereoSetup.RightCameraCalib.K(1, 1) * IMAGE_DOWNSIZE_FACTOR;
        stereoSetup.RightCameraCalib.K(0, 2) = stereoSetup.RightCameraCalib.K(0, 2) * IMAGE_DOWNSIZE_FACTOR;
        stereoSetup.RightCameraCalib.K(1, 2) = stereoSetup.RightCameraCalib.K(1, 2) * IMAGE_DOWNSIZE_FACTOR;

        // construct the reconstruction system (pun intended)
        m_ReconstructionSystem = std::make_unique<System::ReconstructionSystem>(m_Config, stereoSetup);

        // calib data will be loaded by now - expecting a constant stream of stereo messages at this point
        // these are in the server's data queue
        CVNetwork::Message::StereoMessage message;
        Pipeline::StereoFrame frame;

        m_ProcessingStarted = true;

        while (!m_UserRequestedToQuit)
        {
            if (m_ReconstructionServer->GetNextStereoDataFromQueue(message))
            {
                //std::cout << "\nProcessing frame #" << n << " in queue" << std::endl;

                // got a stereo message from the client process with 3D reconstruct
                frame = Utility::MessageConverter::ConvertStereoMessage(message);
                frame.ID = m_NumFramesProcessed;
                
                // downsize the image to half its size
                cv::resize(frame.LeftImage, frame.LeftImage, cv::Size(frame.LeftImage.cols * IMAGE_DOWNSIZE_FACTOR, frame.LeftImage.rows * IMAGE_DOWNSIZE_FACTOR));
                cv::resize(frame.RightImage, frame.RightImage, cv::Size(frame.RightImage.cols * IMAGE_DOWNSIZE_FACTOR, frame.RightImage.rows * IMAGE_DOWNSIZE_FACTOR));

                // submit to reconstruction system for processing
                m_ReconstructionSystem->ProcessStereoFrame(frame);

                m_NumFramesProcessed++;
            }
        }

        // wait for networking read thread to join
        m_ReconstructionServer->StopServer();

        // save current built map (point cloud) to file
        std::cout << "\n\nSaving current built map to disk..." << std::endl;

        /*
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud { new pcl::PointCloud<pcl::PointXYZRGB>() };
        m_ReconstructionSystem->GetCurrentBuiltMap(cloud);

        pcl::io::savePCDFileBinary("full_cloud.pcd", *cloud);
        */
        
        m_ReconstructionSystem->RequestShutdown();
    }
}
