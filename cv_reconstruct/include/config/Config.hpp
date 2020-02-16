//
// Config.hpp
// Stores the config for the reconstruction server
//

#ifndef MASTER_THESIS_CONFIG_HPP
#define MASTER_THESIS_CONFIG_HPP

#include <string>

#include "reconstruct/Reconstruct3D.hpp"

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
            bool ShouldRectifyImages{true};
            Reconstruct::StereoBlockMatcherType BlockMatcherType{ Reconstruct::StereoBlockMatcherType::STEREO_BLOCK_MATCHER };
            int NumDisparities{16};
            int WindowSize{21};

        } Reconstruction;

        // Point cloud post processing
        struct PointCloudPostProcess
        {
            double OutlierStdDevThreshold { 1.0 };
            int OutlierMinK { 50 };

        } PointCloudPostProcess;
    };
}

#endif //MASTER_THESIS_CONFIG_HPP
