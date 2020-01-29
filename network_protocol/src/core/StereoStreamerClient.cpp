//
// StereoStreamerClient.cpp
// The client for streaming stereo image data and pose of the robot to the reconstruction server
//

#include "StereoStreamerClient.hpp"


namespace CVNetwork
{
    namespace Clients
    {
        // Constructor
        StereoStreamerClient::StereoStreamerClient(const Actor::RobotStreamer* robotStreamer) : m_RobotStreamer(robotStreamer)
        {

        }

        // Destructor
        StereoStreamerClient::~StereoStreamerClient()
        {
            // close and wait for thread to join
            m_IsRunning = false;
            m_Thread.join();

            // close connection in the stereo stream
            m_StereoStream.CloseConnection();
        }

        // Connect to reconstruct server
        bool StereoStreamerClient::ConnectToReconstructServer(const std::string& ip, int port) {
            return m_StereoStream.Connect(ip, port);
        }

        // Run the client indefinitely until requested to close or connection closed
        void StereoStreamerClient::Run()
        {
            m_IsRunning = true;
            m_Thread = std::thread(&StereoStreamerClient::RunThread, this);
        }

        // Add a stereo data message to the queue
        void StereoStreamerClient::AddStereoDataToQueue(const Message::StereoMessage &message)
        {
            m_QueueMutex.lock();
            m_DataQueue.push(message);
            m_QueueMutex.unlock();
        }

        // Main thread loop
        void StereoStreamerClient::RunThread()
        {
            while (m_IsRunning)
            {
                // TODO: initiate the flow by sending server message that flow will begin

                // TODO: get acknowledgement from server. If server requests calib file, send calib

                // main data stream loop
                // read from queue and send any pending messages
                Message::StereoMessage message = GetNextStereoMessageInQueue();
                m_StereoStream.WriteStereoImageData(message);
            }
        }

        // Thread-safe get next message from queue
        Message::StereoMessage StereoStreamerClient::GetNextStereoMessageInQueue()
        {
            m_QueueMutex.lock();

            Message::StereoMessage message = m_DataQueue.front();
            m_DataQueue.pop();

            m_QueueMutex.unlock();

            return message;
        }
    }
}
