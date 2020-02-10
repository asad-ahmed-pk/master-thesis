//
// Config.hpp
// Stores the config for the reconstruction server
//

#ifndef MASTER_THESIS_CONFIG_HPP
#define MASTER_THESIS_CONFIG_HPP

#include <string>

namespace Config
{
    struct Config
    {
        int ServerPort;
        bool ShouldRectifyImages { true };
    };
}

#endif //MASTER_THESIS_CONFIG_HPP
