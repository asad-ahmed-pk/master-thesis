//
// test_server.cpp
// Executable for testing the server functionality of the library
//

#include <iostream>

#include "server/ReconstructionServer.hpp"

int main(int argc, char** argv)
{
    CVNetwork::Servers::ReconstructionServer server;
    server.StartServer();

    std::cin.get();

    return 0;
}
