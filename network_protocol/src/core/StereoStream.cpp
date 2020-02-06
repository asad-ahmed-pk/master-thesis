//
// StereoStream.cpp
// Handles streaming of data from the rover to the cv reconstruct module in the pipeline
//

#include "cv_networking/core/StereoStream.hpp"
#include "cv_networking/message/StereoStreamMessages.hpp"
#include "cv_networking/protocol/ProtocolStream.hpp"

#include <memory>
#include <boost/array.hpp>

namespace CVNetwork
{
    using boost::asio::ip::tcp;

    const int TOTAL_CALIB_ELEMENTS { 36 };

    StereoStream::StereoStream()
    {

    }

    // Connect to server as client
    bool StereoStream::ConnectToServer(const std::string& ip, int port)
    {
        if (m_Socket != nullptr && m_Socket->is_open()) {
            CloseConnection();
        }

        m_Socket = std::make_unique<tcp::socket>(m_IOService);
        m_Socket->connect(tcp::endpoint(boost::asio::ip::address::from_string(ip), port));

        m_IOService.run();

        return (m_Socket != nullptr && m_Socket->is_open());
    }

    // Open socket as a server and wait for the client to connect
    bool StereoStream::StereoStreamClientConnected(int port)
    {
        if (m_Socket != nullptr && m_Socket->is_open()) {
            CloseConnection();
        }

        m_Socket = std::make_unique<tcp::socket>(m_IOService);
        m_IOService.run();

        // wait till a client connects
        tcp::acceptor acceptor(m_IOService, tcp::endpoint(tcp::v4(), port));
        acceptor.accept(*m_Socket);

        // got a connection at this point
        return (m_Socket != nullptr && m_Socket->is_open());
    }

    // Close connection
    void StereoStream::CloseConnection()
    {
        if (m_Socket != nullptr && m_Socket->is_open()) {
            m_Socket->close();
            m_Socket = nullptr;
        }

        m_IOService.stop();
    }

    // Send stereo data
    void StereoStream::WriteStereoImageData(const Message::StereoMessage &message) const
    {
        // write data header
        Protocol::ProtocolStream::WriteHeaderForDataMessage(*m_Socket, Protocol::DataMessageID::DATA_ID_STEREO);

        // write size information for both images
        boost::array<unsigned long, 2> sizeData { static_cast<unsigned long>(message.LeftImageData.size()), static_cast<unsigned long>(message.RightImageData.size()) };
        boost::asio::write(*m_Socket, boost::asio::buffer(sizeData));

        // write left image and right image data
        boost::asio::write(*m_Socket, boost::asio::buffer(message.LeftImageData));
        boost::asio::write(*m_Socket, boost::asio::buffer(message.RightImageData));

        // write location of robot
        boost::array<float, 3> locationData { message.X, message.Y, message.Z };
        boost::asio::write(*m_Socket, boost::asio::buffer(locationData));

        // write orientation of robot
        boost::array<float, 9> rotationData { message.R1, message.R2, message.R3, message.R4, message.R5, message.R6, message.R7, message.R8, message.R9 };
        boost::asio::write(*m_Socket, boost::asio::buffer(rotationData));
    }

    // Read stereo data
    Message::StereoMessage StereoStream::ReadStereoImageData() const
    {
        Message::StereoMessage message{};

        // read in left and right image sizes from header
        boost::array<unsigned long, 2> sizeData{};
        boost::asio::read(*m_Socket, boost::asio::buffer(sizeData));

        message.LeftImageData.resize(sizeData[0]);
        message.RightImageData.resize(sizeData[1]);

        // read in left and right image data
        boost::asio::read(*m_Socket, boost::asio::buffer(message.LeftImageData));
        boost::asio::read(*m_Socket, boost::asio::buffer(message.RightImageData));

        // read pose data
        boost::array<float, 12> poseData{};
        boost::asio::read(*m_Socket, boost::asio::buffer(poseData));

        message.X = poseData[0];
        message.Y = poseData[1];
        message.Z = poseData[2];

        message.R1 = poseData[3];
        message.R2 = poseData[4];
        message.R3 = poseData[5];
        message.R4 = poseData[6];
        message.R5 = poseData[7];
        message.R6 = poseData[8];
        message.R7 = poseData[9];
        message.R8 = poseData[10];
        message.R9 = poseData[11];

        return message;
    }

    // Initiate flow and check if server wants calib data
    bool StereoStream::InitiateStereoAndCheckIfCalibNeeded() const
    {
        // write header ids
        Protocol::ProtocolStream::WriteHeaderForControlMessage(*m_Socket, Protocol::ControlMessageID::CONTROL_ID_ROVER_CONNECT);

        // expect response from server: either begin stereo or ask for calib
        Protocol::ControlMessageID controlMessageID = Protocol::ProtocolStream::ReadControlMessage(*m_Socket);
        return (controlMessageID == Protocol::ControlMessageID::CONTROL_ID_CALIB_REQUEST);
    }

    // Initiate flow by waiting for stereo streamer connection request or direct stereo flow request
    void StereoStream::WaitForConnectAndStartFlow(bool isCalibRequired, Message::StereoCalibMessage &calibMessage) const
    {
        // expect a control message from the robot stereo streamer
        Protocol::ControlMessageID controlMessageID = Protocol::ProtocolStream::ReadControlMessage(*m_Socket);
        if (controlMessageID == Protocol::ControlMessageID::CONTROL_ID_ROVER_CONNECT)
        {
            // robot connected - now request for calib if required
            if (isCalibRequired) {
                Protocol::ProtocolStream::WriteHeaderForControlMessage(*m_Socket, Protocol::ControlMessageID::CONTROL_ID_CALIB_REQUEST);
                ReadCalibData(calibMessage);
            }

            // send control message to start stereo stream
            Protocol::ProtocolStream::WriteHeaderForControlMessage(*m_Socket, Protocol::ControlMessageID::CONTROL_ID_BEGIN_STEREO_DATA_STREAM);
        }
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
        data[8] = message.d15;
        data[9] = message.d16;
        data[10] = message.d17;
        data[11] = message.d18;

        // right cam intrinsics
        data[12] = message.fx2;
        data[13] = message.fy2;
        data[14] = message.cx2;
        data[15] = message.cy2;
        data[16] = message.d21;
        data[17] = message.d22;
        data[18] = message.d23;
        data[19] = message.d24;
        data[20] = message.d25;
        data[21] = message.d26;
        data[22] = message.d27;
        data[23] = message.d28;

        // T
        data[24] = message.t1;
        data[25] = message.t2;
        data[26] = message.t3;

        // R
        data[27] = message.r1;
        data[28] = message.r2;
        data[29] = message.r3;
        data[30] = message.r4;
        data[31] = message.r5;
        data[32] = message.r6;
        data[33] = message.r7;
        data[34] = message.r8;
        data[35] = message.r9;

        // write to socket
        boost::asio::write(*m_Socket, boost::asio::buffer(data));
    }

    // Read calib data from socket
    void StereoStream::ReadCalibData(Message::StereoCalibMessage &message) const
    {
        // read header for calib data
        Protocol::DataMessageID dataMessageID = Protocol::ProtocolStream::ReadDataMessage(*m_Socket);

        // read all calib float data into buffer
        boost::array<float, TOTAL_CALIB_ELEMENTS> data{};
        boost::asio::read(*m_Socket, boost::asio::buffer(data));

        // read left cam intrinsics
        message.fx1 = data[0];
        message.fy1 = data[1];
        message.cx1 = data[2];
        message.cy1 = data[3];
        message.d11 = data[4];
        message.d12 = data[5];
        message.d13 = data[6];
        message.d14 = data[7];
        message.d15 = data[8];
        message.d16 = data[9];
        message.d17 = data[10];
        message.d18 = data[11];

        // read right cam intrinsics
        message.fx2 = data[12];
        message.fy2 = data[13];
        message.cx2 = data[14];
        message.cy2 = data[15];
        message.d21 = data[16];
        message.d22 = data[17];
        message.d23 = data[18];
        message.d24 = data[19];
        message.d25 = data[20];
        message.d26 = data[21];
        message.d27 = data[22];
        message.d28 = data[23];

        // read T
        message.t1 = data[24];
        message.t2 = data[25];
        message.t3 = data[26];

        // read R
        message.r1 = data[27];
        message.r2 = data[28];
        message.r3 = data[29];
        message.r4 = data[30];
        message.r5 = data[31];
        message.r6 = data[32];
        message.r7 = data[33];
        message.r8 = data[34];
        message.r9 = data[35];
    }

    // Get next message from the protocol stream
    void StereoStream::GetNextMessage(Protocol::HeaderID &headerID, Protocol::ControlMessageID &controlMessageID, Protocol::DataMessageID &dataMessageID) {
        Protocol::ProtocolStream::ReadNextMessage(*m_Socket, headerID, controlMessageID, dataMessageID);
    }

    // Check if data available
    bool StereoStream::IsDataAvailableToRead() const {
        return (m_Socket->available() > 0);
    }
}
