//
// StereoStream.cpp
// Handles streaming of data from the rover to the cv reconstruct module in the pipeline
//

#include "StereoStream.hpp"
#include "message/StereoStreamMessages.hpp"

#include <string>
#include <memory>

namespace CVNetwork
{
    using boost::asio::ip::tcp;

    StereoStream::StereoStream()
    {

    }

    // Connect to ip and port
    bool StereoStream::Connect(const std::string& ip, int port)
    {
        if (m_Socket->is_open()) {
            CloseConnection();
        }

        // resolve address of server
        const std::string address { ip + ":" + std::to_string(port) };
        tcp::resolver resolver(m_IOService);
        tcp::resolver::iterator results = resolver.resolve(address);

        m_Socket = std::make_unique<tcp::socket>(m_IOService);
        boost::asio::connect(*m_Socket, results);

        return (m_Socket != nullptr && m_Socket->is_open());
    }

    // Close connection
    void StereoStream::CloseConnection()
    {
        if (m_Socket->is_open()) {
            m_Socket->close();
            m_Socket = nullptr;
        }

        m_IOService.stop();
    }

    // Send stereo data
    void StereoStream::WriteStereoImageData(const Message::StereoMessage &message) const
    {
        // TODO: write message to socket according to protocol specification
    }

    // Read stereo data
    Message::StereoMessage StereoStream::ReadStereoImageData() const
    {
        Message::StereoMessage message;

        // TODO: read from socket according to protocol specification and convert to struct message

        return message;
    }
}
