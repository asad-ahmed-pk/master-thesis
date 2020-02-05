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
#define CONFIG_DEFAULT_PATH "resources/config/config_default.json"
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
        config.ServerPort = serverConfig["port"];

        return config;
    }
}