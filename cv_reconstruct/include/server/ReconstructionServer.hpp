//
// ReconstructionServer.hpp
// The reconstruction server that reads from the stereo stream and continuously reconstructs the scene
// Connects to the visualisation module as a client and streams the reconstructed point clouds
//

#ifndef MASTER_THESIS_RECONSTRUCTIONSERVER_HPP
#define MASTER_THESIS_RECONSTRUCTIONSERVER_HPP

#include <string>
#include <memory>
#include <thread>

#include "ServerUserInterface.hpp"
#include "reconstruct/ReconstructStatusCode.hpp"
#include "config/Config.hpp"
#include "camera/CameraCalib.hpp"
#include "pipeline/ReconstructionPipeline.hpp"
#include "cv_networking/server/ReconstructionServer.hpp"

namespace Server
{
    class ReconstructionServer
    {
    public:
        /// Create an instance of a server that expects a connection from a single client
        /// The server will configure parameters from the JSON config file
        ReconstructionServer();

        /// Server destructor
        ~ReconstructionServer();

        /// Run the server until client disconnects. This is a blocking call.
        /// \return Returns the status code to indicate the cause / error for returning
        ReconstructServerStatusCode Run();

    private:
        void RunProcessingThread();

    private:
        std::atomic<bool> m_UserRequestedToQuit { false };
        std::atomic<bool> m_ProcessingStarted { false };
        std::atomic<long> m_NumFramesProcessed { 0 };

        ServerUserInterface m_UserInterface;
        std::thread m_ProcessingThread;

    private:
        Config::Config m_Config;
        std::unique_ptr<Pipeline::ReconstructionPipeline> m_ReconstructionPipeline;
        std::unique_ptr<Camera::Calib::StereoCalib> m_Calib { nullptr };
        std::unique_ptr<CVNetwork::Servers::ReconstructionServer> m_ReconstructionServer { nullptr };
    };
}

#endif //MASTER_THESIS_RECONSTRUCTIONSERVER_HPP
