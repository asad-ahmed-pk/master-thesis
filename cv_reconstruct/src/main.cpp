//
// main.cpp
// Main executable and entry point for the CV reconstruction server
//

#include "server/ReconstructionServer.hpp"
#include "server/ServerUserInterface.hpp"

int main(int argc, char** argv)
{
    Server::ReconstructServerStatusCode status;
    Server::ReconstructionServer server;

    status = server.Run();

    return 0;
}
