//
// ReconstructionServer.cpp
// Server responsible for 3D reconstruction from the stereo stream from the robot
//

#include "cv_networking/server/ReconstructionServer.hpp"

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
            // close and wait for thread to join
            if (m_Thread.joinable())
            {
                m_IsRunning = false;
                m_Thread.join();

                // close connection in the stereo stream
                m_StereoStream.CloseConnection();

#ifndef NDEBUG
                std::cout << "\nShut down server" << std::endl;
#endif
            }
        }

        // Start server
        void ReconstructionServer::StartServer()
        {
#ifndef NDEBUG
            std::cout << "\nStarting server" << std::endl;
#endif

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
#ifndef NDEBUG
            std::cout << "\nMain server thread running... Waiting for a stereo streaming client to connect on port " << m_Port << std::endl;
#endif

            // open socket and listen on port for the stereo streamer to connect
            if (m_StereoStream.StereoStreamClientConnected(m_Port))
            {
#ifndef NDEBUG
                std::cout << "\nClient connected on port " << m_Port << std::endl;
#endif

                // start the flow: either ask for calib data or begin the stereo stream
                Message::StereoCalibMessage calibMessage{};
                m_StereoStream.WaitForConnectAndStartFlow(m_IsCalibRequired, calibMessage);

                if (m_IsCalibRequired) {
#ifndef NDEBUG
                    //std::cout << "\nReceived calib from client: " << calibMessage.fx1 << " " << calibMessage.fx2 << std::endl;
#endif
                    m_IsCalibAvailable = true;
                    m_CalibMessage = calibMessage;
                }

                // run the main server loop
                RunMainServerLoop();
            }
        }

        // Calib check
        bool ReconstructionServer::IsCalibAvailable() const {
            return m_IsCalibAvailable;
        }

        // Get calib
        Message::StereoCalibMessage ReconstructionServer::GetCalibMessage() const {
            return m_CalibMessage;
        }

        // Main server loop
        void ReconstructionServer::RunMainServerLoop()
        {
#ifndef NDEBUG
            std::cout << "\nRunning main server loop. Reading messages from client" << std::endl;
#endif

            Protocol::HeaderID  headerID;
            Protocol::DataMessageID dataMessageID;
            Protocol::ControlMessageID controlMessageID;

            while (m_IsRunning)
            {
                // get the next message and respond to it if there is data
                if (m_StereoStream.IsDataAvailableToRead())
                {
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

                // sleep so client thread can process
                std::this_thread::sleep_for(std::chrono::milliseconds (100));
            }

            m_StereoStream.CloseConnection();
        }

        // Process data message
        void ReconstructionServer::ProcessDataMessage(CVNetwork::Protocol::DataMessageID dataMessageID)
        {
            switch (dataMessageID)
            {
                // got a stereo data message - add to queue
                case Protocol::DataMessageID::DATA_ID_STEREO: {
                    Message::StereoMessage message = m_StereoStream.ReadStereoImageData();

#ifndef NDEBUG
                    //std::cout << "\nStereo data read from client" << std::endl;
                    //std::cout << "\nX, Y, Z = " << message.X << ", " << message.Y << ", " << message.Z << std::endl;
#endif

                    m_Mutex.lock();
                    m_DataQueue.push(message);
                    m_Mutex.unlock();

#ifndef NDEBUG
                    //std::cout << "\nAdded to queue" << std::endl;
#endif

                    break;
                }

                case Protocol::DataMessageID::DATA_ID_CALIB:
                    break;
            }
        }

        // Get stereo data from queue
        bool ReconstructionServer::GetNextStereoDataFromQueue(Message::StereoMessage &message)
        {
            m_Mutex.lock();

            if (!m_DataQueue.empty())
            {
                message = m_DataQueue.front();
                m_DataQueue.pop();
                m_Mutex.unlock();

                return true;
            }

            m_Mutex.unlock();
            return false;
        }
    }
}
