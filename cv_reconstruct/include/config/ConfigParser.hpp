//
// ConfigParser.hpp
// Parses JSON config for the server settings
//

#ifndef MASTER_THESIS_CONFIGPARSER_HPP
#define MASTER_THESIS_CONFIGPARSER_HPP

#include "Config.hpp"

namespace Config
{
    class ConfigParser
    {
    public:
        ConfigParser() = default;
        ~ConfigParser() = default;

        /// Parse and return the config file in the executable directory.
        /// If no config.json file exists, the default file will be copied
        /// \return The parsed config
        Config ParseConfig();
    };
}

#endif //MASTER_THESIS_CONFIGPARSER_HPP
