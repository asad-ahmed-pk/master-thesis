//
// StereoStreamMessages.hpp
// Contains structs defining messages for the stereo stream
//

#ifndef NETWORK_PROTOCOL_STEREOSTREAMMESSAGES_HPP
#define NETWORK_PROTOCOL_STEREOSTREAMMESSAGES_HPP

namespace CVNetwork
{
    namespace Message
    {
        // Message with stereo images and pose of robot when images were taken
        struct StereoMessage
        {
            unsigned char* LeftImageData;
            unsigned char* RightStereoImageData;

            float X; float Y; float Z;

            float R1, R2, R3;
            float R4, R5, R6;
            float R7, R8, R9;
        };

        // Message with stereo calibration information
        struct StereoCalibMessage
        {
            // Left camera intrinsics
            float fx1; float fy1;
            float cx1; float cy1;
            float d11, d12, d13, d14;

            // Right camera intrinsics
            float fx2; float fy2;
            float cx2; float cy2;
            float d21, d22, d23, d24;

            // Relative extrinsics
            float t1; float t2; float t3;
            float r1, r2, r3;
            float r4, r5, r6;
            float r7, r8, r9;
        };
    }
}

#endif //NETWORK_PROTOCOL_STEREOSTREAMMESSAGES_HPP
