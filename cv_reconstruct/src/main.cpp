//
// main.cpp
// Main executable and entry point for the CV reconstruction server
//

#include "server/ReconstructionServer.hpp"

int main(int argc, char** argv)
{
    Reconstruct::ReconstructServerStatusCode status;
    Reconstruct::ReconstructionServer server;

    status = server.Run();

    return 0;
}
