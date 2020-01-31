//
// ProtocolStream.hpp
// Static class with functions for common protocol read / write operations for streams
//

#ifndef NETWORK_PROTOCOL_PROTOCOLSTREAM_HPP
#define NETWORK_PROTOCOL_PROTOCOLSTREAM_HPP

#include <boost/asio.hpp>

#include "protocol.hpp"

namespace CVNetwork
{
    namespace Protocol
    {
        class ProtocolStream
        {
        public:
            /// Write bytes for sending a control message of the given control type
            /// \param socket A reference to the socket to which bytes will be written
            /// \param controlMessageID The ID to identify the type of control message being sent
            static void WriteHeaderForControlMessage(boost::asio::ip::tcp::socket& socket, ControlMessageID controlMessageID);

            /// Read the control message and parse the ID
            /// \param socket The socket to read from
            /// \return Returns the parsed control message id
            static ControlMessageID ReadControlMessage(boost::asio::ip::tcp::socket& socket);

            /// Read the data message and parse the ID of the type of data being sent
            /// \param socket The socket to read from
            /// \return Returns the parse data message id
            static DataMessageID ReadDataMessage(boost::asio::ip::tcp::socket& socket);

            /// Write bytes for sending a data message
            /// \param socket A reference to the socket to which bytes will be written
            static void WriteHeaderForDataMessage(boost::asio::ip::tcp::socket& socket, DataMessageID dataMessageID);
        };
    }
}

#endif //NETWORK_PROTOCOL_PROTOCOLSTREAM_HPP
