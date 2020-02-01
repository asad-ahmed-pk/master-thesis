//
// ProtocolStream.cpp
// Static class with functions for common protocol read / write operations for streams
//

#include "ProtocolStream.hpp"

#include <boost/array.hpp>

namespace CVNetwork
{
    namespace Protocol
    {
        // Read next message and determine if control or data message
        void ProtocolStream::ReadNextMessage(boost::asio::ip::tcp::socket &socket, HeaderID &headerID, ControlMessageID &controlMessageID, DataMessageID &dataMessageID)
        {
            socket.wait(boost::asio::ip::tcp::socket::wait_read);

            boost::array<int, 2> headerData {};
            boost::asio::read(socket, boost::asio::buffer(headerData));

            // first field: the main header id
            headerID = static_cast<HeaderID>(headerData[0]);

            // depending on type of header id parse the correct message type
            switch (headerID)
            {
                case HEADER_ID_CONTROL:
                    controlMessageID = static_cast<ControlMessageID>(headerData[1]);
                    break;

                case HEADER_ID_DATA:
                    dataMessageID = static_cast<DataMessageID>(headerData[1]);
                    break;
            }
        }

        // Write control message header
        void ProtocolStream::WriteHeaderForControlMessage(boost::asio::ip::tcp::socket &socket, ControlMessageID controlMessageID)
        {
            boost::array<int, 2> data { static_cast<int>(HeaderID::HEADER_ID_CONTROL), static_cast<int>(controlMessageID) };
            boost::asio::write(socket, boost::asio::buffer(data));
        }

        // Read and parse control message ID
        ControlMessageID ProtocolStream::ReadControlMessage(boost::asio::ip::tcp::socket &socket)
        {
            boost::array<int, 2> data{};
            boost::asio::read(socket, boost::asio::buffer(data));

            // first header is message ID, 2nd is the control ID
            ControlMessageID controlMessageID = static_cast<ControlMessageID>(data[1]);
            return controlMessageID;
        }

        DataMessageID ProtocolStream::ReadDataMessage(boost::asio::ip::tcp::socket &socket)
        {
            boost::array<int, 2> data{};
            boost::asio::read(socket, boost::asio::buffer(data));

            // second header is the data id
            DataMessageID dataMessageID = static_cast<DataMessageID >(data[1]);
            return dataMessageID;
        }

        // Write data message header
        void ProtocolStream::WriteHeaderForDataMessage(boost::asio::ip::tcp::socket &socket, DataMessageID dataMessageID)
        {
            boost::array<int, 2> data { static_cast<int>(HeaderID::HEADER_ID_DATA), static_cast<int>(dataMessageID) };
            boost::asio::write(socket, boost::asio::buffer(data));
        }
    }
}
