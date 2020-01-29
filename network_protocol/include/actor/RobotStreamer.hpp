//
// RobotStreamer.hpp
// Represents the interface for the robot stream in the CV pipeline
// The robot streamer streams stereo images, location, and calibration information to the CV reconstructor actor
//

#ifndef NETWORK_PROTOCOL_ROBOTSTREAMER_HPP
#define NETWORK_PROTOCOL_ROBOTSTREAMER_HPP

namespace Actor
{
    class RobotStreamer
    {
    public:
        virtual ~RobotStreamer() {};

    };
}

#endif //NETWORK_PROTOCOL_ROBOTSTREAMER_HPP
