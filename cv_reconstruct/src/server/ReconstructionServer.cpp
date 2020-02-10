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

#include <memory>
#include <thread>
#include <chrono>
#include <boost/filesystem.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/highgui/highgui.hpp>

namespace Reconstruct
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
        m_ReconstructionServer = std::make_unique<CVNetwork::Servers::ReconstructionServer>(m_Config.ServerPort, m_Calib == nullptr);
    }

    // Destructor
    ReconstructionServer::~ReconstructionServer()
    {

    }

    // Run server
    ReconstructServerStatusCode ReconstructionServer::Run()
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

        // construct the reconstruction module with the calib information
        m_Reconstruction = std::make_unique<Reconstruct3D>(stereoSetup, m_Config.ShouldRectifyImages);

        // calib data will be loaded by now - expecting a constant stream of stereo messages at this point
        // these are in the server's data queue
        CVNetwork::Message::StereoMessage message;
        Reconstruct::StereoFrame frame;

        pcl::PointCloud<pcl::PointXYZRGB> pointCloud;
        pcl::PointCloud<pcl::PointXYZRGB> finalPointCloud;

        long n = 0;

        while (isRunning)
        {
            if (m_ReconstructionServer->GetNextStereoDataFromQueue(message))
            {
                // got a stereo message from the client process with 3D reconstruct
                frame = std::move(Utility::MessageConverter::ConvertStereoMessage(message));
                frame.ID = n;

                // clear all points from temp point cloud
                pointCloud.clear();

                // process this frame
                m_Reconstruction->ProcessFrame(frame, pointCloud);

                // append all points from this point cloud to final point cloud
                for (const auto& p : pointCloud) {
                    finalPointCloud.push_back(p);
                }

                // debugging: save temp point cloud
                pcl::io::savePCDFileBinary("point_cloud_" + std::to_string(frame.ID) + ".pcd", pointCloud);

                std::cout << "\nProcessed frame at " << frame.Translation(0) << ", " << frame.Translation(1) << ", " << frame.Translation(2) << std::endl;
                n++;

                if (n >= 20) {
                    isRunning = false;
                }
            }
        }

        // save point cloud to file for now
        std::cout << "\nServer finished receiving stream. Saving point cloud to file" << std::endl;
        pcl::io::savePCDFileBinary("final_point_cloud.pcd", finalPointCloud);

        return ReconstructServerStatusCode::SERVER_CLIENT_DISCONNECTED;
    }
}
