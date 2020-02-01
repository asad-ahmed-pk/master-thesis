//
// ReconstructionServer.hpp
// Server responsible for 3D reconstruction from the stereo stream from the robot
// Currently there is only single-client support
//

#ifndef NETWORK_PROTOCOL_RECONSTRUCTIONSERVER_HPP
#define NETWORK_PROTOCOL_RECONSTRUCTIONSERVER_HPP

#include <queue>
#include <thread>
#include <string>
#include <mutex>

#include "../../src/core/StereoStream.hpp"
#include "message/StereoStreamMessages.hpp"

namespace CVNetwork
{
    namespace Servers
    {
        class ReconstructionServer
        {
        public:
            /// Create an instance of the server with the given address
            /// \param port The port that the server will listen on. Default is 7000.
            /// \param isCalibRequired Set to true if client should provide calib data after connecting. Default is true.
            ReconstructionServer(int port = 7000, bool isCalibRequired = true);

            ~ReconstructionServer();

            /// Start listening for a connection
            void StartServer();

            /// Shut down server
            void StopServer();

        private:
            void ServerMainThread();
            void RunMainServerLoop();
            void ProcessDataMessage(Protocol::DataMessageID dataMessageID);

        private:
            int m_Port;

            bool m_IsRunning { false };
            bool m_IsCalibRequired;

            StereoStream m_StereoStream;

            std::queue<Message::StereoMessage> m_DataQueue;
            std::thread m_Thread;
            std::mutex m_Mutex;
        };
    }
}

#endif //NETWORK_PROTOCOL_RECONSTRUCTIONSERVER_HPP
