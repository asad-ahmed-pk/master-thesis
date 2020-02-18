//
// ReconstructServerError.hpp
// Status codes for the reconstruction server
//

#ifndef MASTER_THESIS_RECONSTRUCTSTATUSCODE_HPP
#define MASTER_THESIS_RECONSTRUCTSTATUSCODE_HPP

namespace Server
{
    enum ReconstructServerStatusCode
    {
        SERVER_CLIENT_DISCONNECTED,
        SERVER_CONNECTION_TIMEOUT,
        SERVER_UNKNOWN_EXCEPTION
    };
}

#endif //MASTER_THESIS_RECONSTRUCTSTATUSCODE_HPP
