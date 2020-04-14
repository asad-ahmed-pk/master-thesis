//
// Config.hpp
// Stores the config for the reconstruction server
//

#ifndef MASTER_THESIS_CONFIG_HPP
#define MASTER_THESIS_CONFIG_HPP

#include <string>

#include "point_cloud/point_cloud_constants.hpp"
#include "reconstruct/Reconstruct3DTypes.hpp"

namespace Config
{
    struct Config
    {
        // Server
        struct Server
        {
            int ServerPort;

        } Server;

        // 3D Reconstruction
        struct Reconstruction
        {
            bool ShouldRectifyImages { true };
            Reconstruct::StereoBlockMatcherType BlockMatcherType { Reconstruct::StereoBlockMatcherType::STEREO_BLOCK_MATCHER };

            struct SBM {
                int NumDisparities { 16 };
                int WindowSize { 21 };
            } SBM;

            struct SGBM {
                int NumDisparities { 16 };
                int BlockSize { 128 };
                int UniquenessRatio { 2 };
                int SpeckleRange { 2 };
                int SpeckleWindowSize { 100 };
                int PreFilterCap { 10 };
                int MinDisparity { 0 };
            } SGBM;

        } Reconstruction;
    };
}

#endif //MASTER_THESIS_CONFIG_HPP
