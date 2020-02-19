//
// FeatureExtractionConfig.hpp
// Struct that encapsulates parameters for feature extraction for the Feature extractor
// Contains settings for the keypoint detector and feature extractor
//

#ifndef MASTER_THESIS_FEATUREEXTRACTIONCONFIG_HPP
#define MASTER_THESIS_FEATUREEXTRACTIONCONFIG_HPP

#include "config/Config.hpp"

namespace PointCloud
{
    struct FeatureExtractionConfig
    {
    public:
        /// Construct the default instance of the config with default params (usually most set to 0)
        FeatureExtractionConfig() = default;

        ~FeatureExtractionConfig() = default;

        /// Construct the config from the general config
        /// \param generalConfig The parsed general config that contains the config for point cloud processing
        explicit FeatureExtractionConfig(const Config::Config& generalConfig);

    public:
        struct Normal {
            float Radius { 0.0f };
        } NormalParams;



    };
}

#endif //MASTER_THESIS_FEATUREEXTRACTIONCONFIG_HPP
