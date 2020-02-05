//
// ReconstructionServer.hpp
// The reconstruction server that reads from the stereo stream and continuously reconstructs the scene
// Connects to the visualisation module as a client and streams the reconstructed point clouds
//

#include "reconstruct/ReconstructionServer.hpp"
#include "cv_networking/message/StereoStreamMessages.hpp"
#include "cv_networking/server/ReconstructionServer.hpp"

namespace Reconstruct
{
    // Constructor
    ReconstructionServer::ReconstructionServer(int port) : m_Port(port)
    {

    }

    // Destructor
    ReconstructionServer::~ReconstructionServer()
    {

    }

    // Run server
    ReconstructServerStatusCode ReconstructionServer::Run()
    {
        // TODO: Run server using cv_networking lib
    }
}
