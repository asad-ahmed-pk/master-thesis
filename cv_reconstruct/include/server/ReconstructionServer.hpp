//
// ReconstructionServer.hpp
// The reconstruction server that reads from the stereo stream and continuously reconstructs the scene
// Connects to the visualisation module as a client and streams the reconstructed point clouds
//

#ifndef MASTER_THESIS_RECONSTRUCTIONSERVER_HPP
#define MASTER_THESIS_RECONSTRUCTIONSERVER_HPP

#include <string>
#include <memory>

#include "reconstruct/ReconstructStatusCode.hpp"
#include "config/Config.hpp"
#include "camera/CameraCalib.hpp"
#include "reconstruct/Reconstruct3D.hpp"

#include "cv_networking/server/ReconstructionServer.hpp"

namespace Reconstruct
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
        Config::Config m_Config;
        std::unique_ptr<Reconstruct3D> m_Reconstruction;
        std::unique_ptr<Camera::Calib::StereoCalib> m_Calib { nullptr };
        std::unique_ptr<CVNetwork::Servers::ReconstructionServer> m_ReconstructionServer { nullptr };
    };
}

#endif //MASTER_THESIS_RECONSTRUCTIONSERVER_HPP
