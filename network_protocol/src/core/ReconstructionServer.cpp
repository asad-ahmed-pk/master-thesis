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
            m_IsRunning = false;        // setting to false will cause loop to exit in RunMainServerLoop()
        }

        // The main server thread
        void ReconstructionServer::ServerMainThread()
        {
            std::cout << "\nMain server thread running... Waiting for a stereo streaming client to connect on port " << m_Port << std::endl;

            // open socket and listen on port for the stereo streamer to connect
            if (m_StereoStream.StereoStreamClientConnected(m_Port))
            {
                std::cout << "\nClient connected on port " << m_Port << std::endl;

                // start the flow: either ask for calib data or begin the stereo stream
                Message::StereoCalibMessage calibMessage{};
                m_StereoStream.WaitForConnectAndStartFlow(m_IsCalibRequired, calibMessage);

                if (m_IsCalibRequired) {
                    std::cout << "\nReceived calib from client: " << calibMessage.fx1 << " " << calibMessage.fx2 << std::endl;
                }

                // run the main server loop
                RunMainServerLoop();
            }
        }

        // Main server loop
        void ReconstructionServer::RunMainServerLoop()
        {
            std::cout << "\nRunning main server loop. Reading messages from client" << std::endl;

            Protocol::HeaderID  headerID;
            Protocol::DataMessageID dataMessageID;
            Protocol::ControlMessageID controlMessageID;

            while (m_IsRunning)
            {
                // get the next message and respond to it
                m_StereoStream.GetNextMessage(headerID, controlMessageID, dataMessageID);

                switch (headerID)
                {
                    // got a control message
                    case Protocol::HeaderID::HEADER_ID_CONTROL:
                        // TODO: process control message (can be exit message)
                        break;

                    // got a data message
                    case Protocol::HeaderID::HEADER_ID_DATA:
                        ProcessDataMessage(dataMessageID);
                        break;
                }
            }
        }

        // Process data message
        void ReconstructionServer::ProcessDataMessage(CVNetwork::Protocol::DataMessageID dataMessageID)
        {
            switch (dataMessageID)
            {
                // got a stereo data message - add to queue
                case Protocol::DataMessageID::DATA_ID_STEREO: {
                    Message::StereoMessage message = m_StereoStream.ReadStereoImageData();

                    m_Mutex.lock();
                    m_DataQueue.push(message);
                    m_Mutex.unlock();

                    break;
                }

                case Protocol::DataMessageID::DATA_ID_CALIB:
                    break;
            }
        }
    }
}
