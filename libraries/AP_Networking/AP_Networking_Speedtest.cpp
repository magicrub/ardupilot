
#include "AP_Networking_Speedtest.h"

#if AP_NETWORKING_SPEEDTEST_ENABLED
#include <AP_Math/AP_Math.h> // for MIN()

#ifndef AP_NETWORKING_SPEEDTEST_DEFAULT_PORT
#define AP_NETWORKING_SPEEDTEST_DEFAULT_PORT   5555
#endif

#ifndef AP_NETWORKING_SPEEDTEST_UDP_MAX_PACKET_SIZE
#define AP_NETWORKING_SPEEDTEST_UDP_MAX_PACKET_SIZE AP_NETWORKING_ETHERNET_UDP_PAYLOAD_MAX_SIZE
#endif

const AP_Param::GroupInfo AP_Networking_Speedtest::var_info[] = {

    // @Param: PORT
    // @DisplayName: PORT
    // @Description: PORT
    // @Range: 1 65535
    // @User: Standard
    AP_GROUPINFO("PORT", 1, AP_Networking_Speedtest, _params.port, AP_NETWORKING_SPEEDTEST_DEFAULT_PORT),

    AP_GROUPINFO("DST_ADDR0", 2,  AP_Networking_Speedtest,    _params.ip[0],   255),
    AP_GROUPINFO("DST_ADDR1", 3,  AP_Networking_Speedtest,    _params.ip[1],   255),
    AP_GROUPINFO("DST_ADDR2", 4,  AP_Networking_Speedtest,    _params.ip[2],   255),
    AP_GROUPINFO("DST_ADDR3", 5,  AP_Networking_Speedtest,    _params.ip[3],   255),

    AP_GROUPINFO("COUNT", 6, AP_Networking_Speedtest, _params.count, 200),
    AP_GROUPINFO("SIZE", 7, AP_Networking_Speedtest, _params.payload_size, AP_NETWORKING_SPEEDTEST_UDP_MAX_PACKET_SIZE),
    AP_GROUPINFO("CONTENT", 8, AP_Networking_Speedtest, _params.content, -1),
    

    AP_GROUPEND
};

extern const AP_HAL::HAL& hal;

/// Constructor
AP_Networking_Speedtest::AP_Networking_Speedtest(AP_Networking &front, AP_Networking::AP_Networking_State &state,  AP_Networking_Params &params) :
    AP_Networking_Backend(front, state, params)
{
    AP_Param::setup_object_defaults(this, var_info);
    _state.var_info = var_info;
}

void AP_Networking_Speedtest::init()
{
    if (_pcb == nullptr) {
        _pcb = udp_new_ip_type(IPADDR_TYPE_ANY);
        if (_pcb != nullptr) {
            // allow to sending broadcast packets
            ip_set_option(_pcb, SOF_BROADCAST);
        }
    }
}

void AP_Networking_Speedtest::update()
{
    if (_pcb == nullptr) {
        return;
    }

    const uint32_t payload_size = constrain_int32(_params.payload_size.get(), 0, 0xFFFF);
    if (payload_size == 0) {
        return;
    }

    uint8_t payload[payload_size];

    // fill the payload
    if (_params.content.get() == -1) {
        uint16_t offset = 0;
        const char* str = "ArduPilot Networking Speedtest";
        const uint16_t str_len = strlen(str);
        while (offset < payload_size-1) {
            const uint16_t offset_len = MIN(str_len, payload_size-offset-1);
            memcpy(&payload[offset], str, offset_len);
            offset += offset_len;
        }

    } else if (_params.content.get() >= 0) {
        memset(payload, (uint8_t)_params.content.get(), payload_size);

    } else {
        // do not initialize payload[], just send whatever crap/noise thats on the stack
    }

    IP4_ADDR(&_ip4_addr, _params.ip[0], _params.ip[1], _params.ip[2], _params.ip[3]);

    for (int32_t i=0; i<_params.count.get(); i++) {
        // if we were able to queue a packet to send, try to queue one more to keep the queue full
        if (AP_Networking::send_udp(_pcb, _ip4_addr, _params.port, payload, payload_size) <= 0) {
            return;
        }
    }
}

#endif // AP_NETWORKING_SPEEDTEST_ENABLED

