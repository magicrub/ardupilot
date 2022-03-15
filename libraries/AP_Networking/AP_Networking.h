
#pragma once

#include <AP_HAL/AP_HAL.h>


#ifndef HAL_ENABLE_NETWORKING
#define HAL_ENABLE_NETWORKING 1
#endif

#if HAL_ENABLE_NETWORKING

#include <AP_Param/AP_Param.h>


#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include "lwipthread.h"
#else

#if BYTE_ORDER == LITTLE_ENDIAN
#define IP4_ADDR_VALUE(a,b,c,d)        \
        (((uint32_t)((d) & 0xff) << 24) | \
         ((uint32_t)((c) & 0xff) << 16) | \
         ((uint32_t)((b) & 0xff) << 8)  | \
          (uint32_t)((a) & 0xff))
#else
#define IP4_ADDR_VALUE(a,b,c,d)        \
        (((uint32_t)((a) & 0xff) << 24) | \
         ((uint32_t)((b) & 0xff) << 16) | \
         ((uint32_t)((c) & 0xff) << 8)  | \
          (uint32_t)((d) & 0xff))
#endif
#endif


#ifndef AP_NETWORKING_HAS_THREAD
#define AP_NETWORKING_HAS_THREAD 0
#endif

class AP_Networking {
public:
    AP_Networking();

    /* Do not allow copies */
    AP_Networking(const AP_Networking &other) = delete;
    AP_Networking &operator=(const AP_Networking&) = delete;

    void init();

    void update();

    static AP_Networking *get_singleton(void) {return _singleton; }

    static const struct AP_Param::GroupInfo        var_info[];

    uint32_t get_ip_param() const { return IP4_ADDR_VALUE(_param.ipaddr[0],_param.ipaddr[1],_param.ipaddr[2],_param.ipaddr[3]); }
    //uint32_t get_ip() const { return 0; }

    uint8_t get_netmask_param() const { return _param.netmask; }
    uint32_t get_netmask() const;

    uint32_t get_gateway_param() const {return IP4_ADDR_VALUE(_param.gwaddr[0],_param.gwaddr[1],_param.gwaddr[2],_param.gwaddr[3]); }
    //uint32_t get_ip() const { return 0; }

    bool get_dhcp_enabled() const { return _param.dhcp; }
    
private:
    static AP_Networking *_singleton;

#if AP_NETWORKING_HAS_THREAD
    void thread();
#endif
    
    void apply_errata_for_mac_KSZ9896C();

    struct {
        bool done;
    } _init;

    struct {
        AP_Int16 ipaddr[4];
        AP_Int8 netmask;    // bits to mask. example: (16 == 255.255.0.0) and (24 == 255.255.255.0)
        AP_Int16 gwaddr[4];

        AP_Int8 dhcp;
        AP_Int16 macaddr[6];
    } _param;

    HAL_Semaphore _sem;
};

namespace AP {
    AP_Networking *network();
};

#endif // HAL_ENABLE_NETWORKING
