//
// StereoStreamerClient.hpp
// The client for streaming stereo image data and pose of the robot to the reconstruction server
//

#ifndef NETWORK_PROTOCOL_STEREOSTREAMERCLIENT_HPP
#define NETWORK_PROTOCOL_STEREOSTREAMERCLIENT_HPP

#include <queue>
#include <thread>
#include <string>
#include <mutex>

#include "StereoStream.hpp"
#include "actor/RobotStreamer.hpp"
#include "message/StereoStreamMessages.hpp"

namespace CVNetwork
{
    namespace Clients
    {
        class StereoStreamerClient
        {
        public:
            /// Construct a default instance of the client with a reference to the robot streamer
            /// \param robotStreamer The robot streamer that will provide the stereo stream and calibration
            StereoStreamerClient(const Actor::RobotStreamer* robotStreamer);

            ~StereoStreamerClient();

            /// Establish a connection to the reconstruction server
            /// \param ip The IPv4 address of the server
            /// \param port The port number of the server
            /// \return Returns true if connection has been opened successfully
            bool ConnectToReconstructServer(const std::string& ip, int port);

            /// Run the client on a separate thread
            void Run();

            /// Add a stereo data message to the queue
            /// \param message The stereo data message that will be sent through the stream
            void AddStereoDataToQueue(const Message::StereoMessage& message);

        private:
            void RunThread();
            Message::StereoMessage GetNextStereoMessageInQueue();

        private:
            const Actor::RobotStreamer* m_RobotStreamer;
            std::queue<Message::StereoMessage> m_DataQueue;
            StereoStream m_StereoStream;

            bool m_IsRunning { false };

            std::thread m_Thread;
            std::mutex m_QueueMutex;
        };
    }
}

#endif //NETWORK_PROTOCOL_STEREOSTREAMERCLIENT_HPP
