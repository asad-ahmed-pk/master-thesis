//
// ServerUserInterface.hpp
// User interface for the server. Runs on the main thread and takes input options from the user.
//

#ifndef MASTER_THESIS_SERVERUSERINTERFACE_HPP
#define MASTER_THESIS_SERVERUSERINTERFACE_HPP

#include <map>
#include <string>

#include "server/server_constants.hpp"

namespace Server
{
    class ServerUserInterface
    {
    public:
        ServerUserInterface();

        ~ServerUserInterface() = default;

        /// Get the next option from the user after printing the menu and prompt
        /// \return The selected option of the user
        UserOption GetUserCommandOption() const;

        /// Print the user message that the client has connected
        void PrintClientConnectedMessage() const;

        /// Print the status of the processing
        /// \param messagesProcessed The number of messages processed so far
        /// \param messagesInQueue The number of messages still in the queue
        void PrintStatusMessage(long messagesProcessed, long messagesInQueue) const;

    private:
        void PrintCommandHelp() const;

    private:
        std::map<std::string, UserOption> m_SupportedCommands;
    };
}

#endif //MASTER_THESIS_SERVERUSERINTERFACE_HPP
