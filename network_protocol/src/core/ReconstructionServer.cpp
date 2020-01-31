//
// ReconstructionServer.cpp
// Server responsible for 3D reconstruction from the stereo stream from the robot
//

#include "server/ReconstructionServer.hpp"

#include <iostream>

namespace CVNetwork
{
    namespace Servers
    {
        // Constructor
        ReconstructionServer::ReconstructionServer(int port, bool isCalibRequired) : m_Port(port), m_IsCalibRequired(isCalibRequired)
        {

        }

        // Destructor
        ReconstructionServer::~ReconstructionServer()
        {
            std::cout << "\nDestructor called" << std::endl;

            // close and wait for thread to join
            if (m_Thread.joinable())
            {
                m_IsRunning = false;
                m_Thread.join();

                // close connection in the stereo stream
                m_StereoStream.CloseConnection();

                std::cout << "\nShut down server" << std::endl;
            }
        }

        // Start server
        void ReconstructionServer::StartServer()
        {
            std::cout << "\nStarting server" << std::endl;

            // close socket if open
            m_StereoStream.CloseConnection();

            // create and run thread
            m_Thread = std::thread(&ReconstructionServer::ServerMainThread, this);
            m_IsRunning = true;
        }

        void ReconstructionServer::StopServer() {
            m_IsRunning = false;
        }

        // The main server thread
        void ReconstructionServer::ServerMainThread()
        {
            std::cout << "\nMain server thread running... Waiting for a stereo streaming client to connect on port " << m_Port << std::endl;

            // open socket and listen on port for connections
            if (m_StereoStream.StartListeningForConnection(m_Port))
            {
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
