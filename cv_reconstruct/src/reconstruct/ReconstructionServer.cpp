//
// ReconstructionServer.hpp
// The reconstruction server that reads from the stereo stream and continuously reconstructs the scene
// Connects to the visualisation module as a client and streams the reconstructed point clouds
//

#include "reconstruct/ReconstructionServer.hpp"
#include "config/ConfigParser.hpp"
#include "camera/CameraCalibParser.hpp"
#include "cv_networking/message/StereoStreamMessages.hpp"

#include <memory>
#include <thread>
#include <chrono>
#include <boost/filesystem.hpp>

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
        m_ReconstructionServer = std::make_unique<CVNetwork::Servers::ReconstructionServer>(m_Config.ServerPort, m_Calib != nullptr);
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
                    m_Calib = std::make_unique<Camera::Calib::StereoCalib>(ConvertNetworkCalibMessage(calibMessage));
                    break;
                }
                else {
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
            }
        }

        // construct the reconstruction module with the calib information
        m_Reconstruction = std::make_unique<Reconstruct3D>(*m_Calib);

        // calib data will be loaded by now - expecting a constant stream of stereo messages at this point
        // these are in the server's data queue
        while (isRunning)
        {
            CVNetwork::Message::StereoMessage message;
            if (m_ReconstructionServer->GetNextStereoDataFromQueue(message))
            {
                // TODO: got a stereo message from the client process with 3D reconstruct

            }
        }
    }

    // Convert network calib message
    Camera::Calib::StereoCalib ReconstructionServer::ConvertNetworkCalibMessage(CVNetwork::Message::StereoCalibMessage &calibMessage) const
    {
        // TODO: convert the message that was received from the networking stack to a stereo message record for the reconstruction module
    }
}
