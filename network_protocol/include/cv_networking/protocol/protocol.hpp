//
// protocol.hpp
// Header containing information on protocol IDs and fields
//

#ifndef NETWORK_PROTOCOL_PROTOCOL_HPP
#define NETWORK_PROTOCOL_PROTOCOL_HPP

namespace CVNetwork
{
    namespace Protocol
    {
        /// The message header id type
        enum HeaderID {
            HEADER_ID_CONTROL = 0,
            HEADER_ID_DATA = 1
        };

        /// IDs for control messages to identify the type of the control message
        enum ControlMessageID {
            CONTROL_ID_ROVER_CONNECT = 0,
            CONTROL_ID_CALIB_REQUEST,
            CONTROL_ID_BEGIN_STEREO_DATA_STREAM,
            CONTROL_ID_END_STEREO_DATA_STREAM
        };

        /// IDs for data messages to identify the type of data message
        enum DataMessageID {
            DATA_ID_CALIB = 0,
            DATA_ID_STEREO
        };
    }
}

#endif //NETWORK_PROTOCOL_PROTOCOL_HPP
