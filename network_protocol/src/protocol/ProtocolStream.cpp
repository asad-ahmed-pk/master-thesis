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
        // Write control message header
        void ProtocolStream::WriteHeaderForControlMessage(const boost::asio::ip::tcp::socket &socket, ControlMessageID controlMessageID)
        {
            boost::array<int, 2> data { static_cast<int>(HeaderID::HEADER_ID_CONTROL), static_cast<int>(controlMessageID) };
            boost::asio::write(socket, boost::asio::buffer(data));
        }

        // Read and parse control message ID
        ControlMessageID ProtocolStream::ReadControlMessage(const boost::asio::ip::tcp::socket &socket)
        {
            boost::array<int, 2> data;
            boost::asio::read(socket, data);

            // first header is message ID, 2nd is the control ID
            ControlMessageID controlMessageID = static_cast<ControlMessageID>(data[1]);
            return controlMessageID;
        }

        // Write data message header
        void ProtocolStream::WriteHeaderForDataMessage(const boost::asio::ip::tcp::socket &socket, DataMessageID dataMessageID)
        {
            boost::array<int, 2> data { static_cast<int>(HeaderID::HEADER_ID_DATA), static_cast<int>(dataMessageID) };
            boost::asio::write(socket, boost::asio::buffer(data));
        }
    }
}
