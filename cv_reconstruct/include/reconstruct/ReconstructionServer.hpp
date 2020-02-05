//
// ReconstructionServer.hpp
// The reconstruction server that reads from the stereo stream and continuously reconstructs the scene
// Connects to the visualisation module as a client and streams the reconstructed point clouds
//

#ifndef MASTER_THESIS_RECONSTRUCTIONSERVER_HPP
#define MASTER_THESIS_RECONSTRUCTIONSERVER_HPP

#include <string>

#include "ReconstructStatusCode.hpp"

namespace Reconstruct
{
    class ReconstructionServer
    {
    public:
        /// Create an instance of a server that expects a connection from a single client
        /// \param port (optional) The port of the reconstruction server. Default is 7000.
        explicit ReconstructionServer(int port = 7000);

        ~ReconstructionServer();

        /// Run the server until client disconnects. This is a blocking call.
        /// \return Returns the status code to indicate the cause / error for returning
        ReconstructServerStatusCode Run();

    private:
        int m_Port;
    };
}

#endif //MASTER_THESIS_RECONSTRUCTIONSERVER_HPP
