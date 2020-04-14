//
// ConfigParser.hpp
// Parses JSON config for the server settings
//

#include "config/ConfigParser.hpp"
#include "nlohmann/json.hpp"

#include <iostream>
#include <boost/filesystem.hpp>

namespace Config
{
#define CONFIG_DEFAULT_PATH "../../../cv_reconstruct/resources/config/config_default.json"
#define CONFIG_FILE_PATH "config.json"

    // Parse config
    Config ConfigParser::ParseConfig()
    {
        // check if config exists
        if (!boost::filesystem::exists(CONFIG_FILE_PATH)) {
            boost::filesystem::copy_file(CONFIG_DEFAULT_PATH, CONFIG_FILE_PATH);
        }

        std::ifstream fs(CONFIG_FILE_PATH, std::ios::in);
        nlohmann::json json;
        fs >> json;
        fs.close();

        // extract json into config record
        Config config;
        nlohmann::json serverConfig = json["config"]["server"];

        // server config
        config.Server.ServerPort = serverConfig["port"];

        // reconstruction config
        nlohmann::json reconstructionConfig = json["config"]["reconstruction"];
        config.Reconstruction.ShouldRectifyImages = reconstructionConfig["requires_rectification"];

        // block matcher parsed into enum
        std::string bmTypeString = reconstructionConfig["block_matcher"];
        if (bmTypeString == "stereo_bm") {
            config.Reconstruction.BlockMatcherType = Reconstruct::StereoBlockMatcherType::STEREO_BLOCK_MATCHER;
        }
        else if (bmTypeString == "stereo_sgbm") {
            config.Reconstruction.BlockMatcherType = Reconstruct::StereoBlockMatcherType::STEREO_SEMI_GLOBAL_BLOCK_MATCHER;
        }
        else {
            config.Reconstruction.BlockMatcherType = Reconstruct::StereoBlockMatcherType::STEREO_BLOCK_MATCHER;
        }

        // SBM stereo matcher
        config.Reconstruction.SBM.WindowSize = reconstructionConfig["SBM"]["window_size"];
        config.Reconstruction.SBM.NumDisparities = reconstructionConfig["SBM"]["num_disparities"];

        // SGBM stereo matcher
        config.Reconstruction.SGBM.BlockSize = reconstructionConfig["SGBM"]["block_size"];
        config.Reconstruction.SGBM.PreFilterCap = reconstructionConfig["SGBM"]["pre_filter_cap"];
        config.Reconstruction.SGBM.SpeckleRange = reconstructionConfig["SGBM"]["speckle_range"];
        config.Reconstruction.SGBM.SpeckleWindowSize = reconstructionConfig["SGBM"]["speckle_window_size"];
        config.Reconstruction.SGBM.UniquenessRatio = reconstructionConfig["SGBM"]["uniqueness_ratio"];
        config.Reconstruction.SGBM.MinDisparity = reconstructionConfig["SGBM"]["min_disparity"];
        config.Reconstruction.SGBM.NumDisparities = reconstructionConfig["SGBM"]["num_disparities"];

        return config;
    }
}
