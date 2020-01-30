//
// ReconstructionServer.cpp
// Server responsible for 3D reconstruction from the stereo stream from the robot
//

#include "ReconstructionServer.hpp"

namespace CVNetwork
{
    namespace Servers
    {
        // Constructor
        ReconstructionServer::ReconstructionServer(const std::string& ip, int port) : m_IP(ip), m_Port(port)
        {

        }

        // Destructor
        ReconstructionServer::~ReconstructionServer()
        {
            // close and wait for thread to join
            m_IsRunning = false;
            m_Thread.join();

            // close connection in the stereo stream
            m_StereoStream.CloseConnection();
        }

        // Start server
        void ReconstructionServer::StartServer()
        {
            // TODO: open socket and listen for connections
        }
    }
}
