//
// ServerUserInterface.hpp
// User interface for the server. Runs on the main thread and takes input options from the user.
//

#include <iostream>
#include <boost/algorithm/string.hpp>

#include "server/ServerUserInterface.hpp"

namespace Server
{
    // Constructor
    ServerUserInterface::ServerUserInterface()
    {
        // init command map
        m_SupportedCommands = std::map<std::string, UserOption> {
                {"print status", USER_OPTION_PRINT_STATUS},
                {"quit",         USER_OPTION_QUIT},
                { "help", USER_OPTION_COMMAND_HELP }
        };
    }

    // Print possible commands
    void ServerUserInterface::PrintCommandHelp() const
    {
        std::cout << "\nCommands:\n";
        std::cout << "print status : Print current status of frames processed";
        std::cout << "\nquit : Shut down the server and save the current accumulated point cloud to disk";
        std::cout << std::endl;
    }

    // Print client connected
    void ServerUserInterface::PrintClientConnectedMessage() const {
        std::cout << "\nClient connected and streaming stereo data" << std::endl;
    }

    // Print status message
    void ServerUserInterface::PrintStatusMessage(long messagesProcessed, long messagesInQueue) const {
        std::cout << "\nProcessed " << messagesProcessed << " frames.";
        std::cout << "\nFrames in queue: " << messagesInQueue << std::endl;
    }

    // Get user option from prompt
    UserOption ServerUserInterface::GetUserCommandOption() const
    {
        std::string line;
        bool valid = false;

        while (true)
        {
            std::cout << "\n~> ";
            std::getline(std::cin, line);
            boost::trim_right(line);
            boost::trim_left(line);

            auto iter = m_SupportedCommands.find(line);
            if (iter != m_SupportedCommands.end())
            {
                if (iter->second == USER_OPTION_COMMAND_HELP) {
                    PrintCommandHelp();
                }
                else {
                    return iter->second;
                }
            }
            else {
                std::cout << "Invalid command \'" << line << "\'. Type \'help\' to see a list of possible commands.\n";
            }
        }
    }
}
