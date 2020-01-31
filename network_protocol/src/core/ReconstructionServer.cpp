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
        ReconstructionServer::ReconstructionServer(const std::string& ip, int port, bool isCalibRequired) : m_IP(ip), m_Port(port), m_IsCalibRequired(isCalibRequired)
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
            // close socket if open
            m_StereoStream.CloseConnection();

            // create and run thread
            m_Thread = std::thread(&ReconstructionServer::ServerMainThread, this);
        }

        // The main server thread
        void ReconstructionServer::ServerMainThread()
        {
            // open socket and listen on port for connections
            if (m_StereoStream.StartListeningForConnection(m_Port))
            {
                m_IsRunning = true;

                // start the flow: either ask for calib data or begin the stereo stream
                Message::StereoCalibMessage calibMessage{};
                m_StereoStream.WaitForConnectAndStartFlow(m_IsCalibRequired, calibMessage);

                // by this point - getting continuous stream of stereo images
                RunStereoLoop();
            }
        }

        // Main stereo loop
        void ReconstructionServer::RunStereoLoop()
        {
            while (m_IsRunning)
            {
                Message::StereoMessage message = m_StereoStream.ReadStereoImageData();

                m_Mutex.lock();
                m_DataQueue.push(message);
                m_Mutex.unlock();
            }
        }
    }
}
