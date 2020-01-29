//
// StereoStream.hpp
// Handles streaming of data from the rover to the cv reconstruct module in the pipeline
//

#ifndef NETWORK_PROTOCOL_STEREOSTREAM_HPP
#define NETWORK_PROTOCOL_STEREOSTREAM_HPP

#include <boost/asio.hpp>
#include <memory>

#include "message/StereoStreamMessages.hpp"

namespace CVNetwork
{
    class Network;

    class StereoStream
    {
    public:
        StereoStream();

        ~StereoStream() {};

        /// Connect to the given ip and port
        /// \param ip The IPv4 IP address
        /// \param port The port of the server
        /// \return Returns true on success
        bool Connect(const std::string& ip, int port);

        /// Close the connection to the server
        void CloseConnection();

        /// Send stereo image through the stream
        /// \param message
        void WriteStereoImageData(const Message::StereoMessage& message) const;

        /// Read stereo image data from the stream
        /// \return Returns the message that was read from the stream
        Message::StereoMessage ReadStereoImageData() const;

    private:
        std::unique_ptr<boost::asio::ip::tcp::socket> m_Socket { nullptr };
        boost::asio::io_service m_IOService;
    };
}

#endif //NETWORK_PROTOCOL_STEREOSTREAM_HPP
