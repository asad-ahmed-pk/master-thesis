//
// StereoStream.cpp
// Handles streaming of data from the rover to the cv reconstruct module in the pipeline
//

#include "StereoStream.hpp"
#include "message/StereoStreamMessages.hpp"
#include "../protocol/ProtocolStream.hpp"

#include <string>
#include <memory>
#include <boost/array.hpp>

namespace CVNetwork
{
    using boost::asio::ip::tcp;

    const int TOTAL_CALIB_ELEMENTS { 28 };

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

    // Initiate flow and check if server wants calib data
    bool StereoStream::InitiateStereoAndCheckIfCalibNeeded() const
    {
        // write header ids
        Protocol::ProtocolStream::WriteHeaderForControlMessage(*m_Socket, Protocol::ControlMessageID::CONTROL_ID_ROVER_CONNECT);

        // expect response from server: either begin stereo or ask for calib
        Protocol::ControlMessageID controlMessageID = Protocol::ProtocolStream::ReadControlMessage(*m_Socket);
        return (controlMessageID == Protocol::ControlMessageID::CONTROL_ID_BEGIN_STEREO_DATA_STREAM);
    }

    // Send calib data to server
    void StereoStream::WriteCalibData(const Message::StereoCalibMessage &message) const
    {
        // write header for data message
        Protocol::ProtocolStream::WriteHeaderForDataMessage(*m_Socket, Protocol::DataMessageID::DATA_ID_CALIB);

        // write out all calib elements
        boost::array<float, TOTAL_CALIB_ELEMENTS> data{};

        // left cam intrinsics
        data[0] = message.fx1;
        data[1] = message.fy1;
        data[2] = message.cx1;
        data[3] = message.cy1;
        data[4] = message.d11;
        data[5] = message.d12;
        data[6] = message.d13;
        data[7] = message.d14;

        // right cam intrinsics
        data[8] = message.fx2;
        data[9] = message.fy2;
        data[10] = message.cx2;
        data[11] = message.cy2;
        data[12] = message.d21;
        data[13] = message.d22;
        data[14] = message.d23;
        data[15] = message.d24;

        // T
        data[16] = message.t1;
        data[17] = message.t2;
        data[18] = message.t3;

        // R
        data[19] = message.r1;
        data[20] = message.r2;
        data[21] = message.r3;
        data[22] = message.r4;
        data[23] = message.r5;
        data[24] = message.r6;
        data[25] = message.r7;
        data[26] = message.r8;
        data[27] = message.r9;
    }
}
