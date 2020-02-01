//
// StereoStream.hpp
// Handles streaming of data from the rover to the cv reconstruct module in the pipeline
//

#ifndef NETWORK_PROTOCOL_STEREOSTREAM_HPP
#define NETWORK_PROTOCOL_STEREOSTREAM_HPP

#include <boost/asio.hpp>
#include <memory>

#include "../protocol/protocol.hpp"
#include "message/StereoStreamMessages.hpp"

namespace CVNetwork
{
    class Network;

    class StereoStream
    {
    public:
        StereoStream();

        ~StereoStream() {};

        /// Connect to the given ip and port of a server as a client
        /// \param ip The IPv4 IP address
        /// \param port The port of the server
        /// \return Returns true on success
        bool ConnectToServer(const std::string& ip, int port);

        /// Open a socket and begin listening for a client
        /// \param port The port that the server is listening on
        /// \return Returns true on success
        bool StereoStreamClientConnected(int port);

        /// Close the connection to the server
        void CloseConnection();

        /// Send stereo image through the stream
        /// \param message
        void WriteStereoImageData(const Message::StereoMessage& message) const;

        /// Read stereo image data from the stream
        /// \return Returns the message that was read from the stream
        Message::StereoMessage ReadStereoImageData() const;

        /// Notify server and get server ready to receive stereo stream. Also checks if calib is needed or not.
        /// \return Returns true if server is requesting calib data. False if ok to start streaming stereo.
        bool InitiateStereoAndCheckIfCalibNeeded() const;

        /// Start the flow by sending the control message, and optionally ask for calib data if required.
        /// \param isCalibRequired If true, calib data will be requested first
        /// \param calibMessage Will be set with the calib data if isCalibRequired is set to true.
        void WaitForConnectAndStartFlow(bool isCalibRequired, Message::StereoCalibMessage& calibMessage) const;

        /// Send calibration data through the socket
        /// \param message The message with the calibration data
        void WriteCalibData(const Message::StereoCalibMessage& message) const;

        /// Read calib data from the socket
        /// \param message Will be filled with calib data that was received
        void ReadCalibData(Message::StereoCalibMessage& message) const;

        /// Get the next message from the socket
        /// \param headerID The type of message (control or data)
        /// \param controlMessageID If control, will be set with the control ID
        /// \param dataMessageID If data, will be set with the data ID
        void GetNextMessage(Protocol::HeaderID& headerID, Protocol::ControlMessageID& controlMessageID, Protocol::DataMessageID& dataMessageID);

    private:
        std::unique_ptr<boost::asio::ip::tcp::socket> m_Socket { nullptr };
        boost::asio::io_service m_IOService;
    };
}

#endif //NETWORK_PROTOCOL_STEREOSTREAM_HPP
