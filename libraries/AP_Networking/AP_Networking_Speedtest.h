
#pragma once

#include "AP_Networking_Backend.h"

#ifndef AP_NETWORKING_SPEEDTEST_ENABLED
#define AP_NETWORKING_SPEEDTEST_ENABLED AP_NETWORKING_ENABLED
#endif

#if AP_NETWORKING_SPEEDTEST_ENABLED

#ifndef AP_NETWORKING_SPEEDTEST_BUFFER_UDP_TX_SIZE
#define AP_NETWORKING_SPEEDTEST_BUFFER_UDP_TX_SIZE 1000
#endif

class AP_Networking_Speedtest  : public AP_Networking_Backend {
public:
    AP_Networking_Speedtest(AP_Networking &front,
                        AP_Networking::AP_Networking_State &state,
                        AP_Networking_Params &params);

    void init() override;

    void update() override;

    static const struct AP_Param::GroupInfo var_info[];

private:
    
    struct {
        AP_Int16 ip[4];
        AP_Int32 port;
        AP_Int16 count;
        AP_Int16 payload_size;
        AP_Int16 content;
    } _params;

    struct udp_pcb *_pcb;
    ip4_addr_t _ip4_addr;

};

#endif // AP_NETWORKING_SPEEDTEST_ENABLED
