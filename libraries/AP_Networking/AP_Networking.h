
#pragma once

#include <AP_HAL/AP_HAL.h>

#if HAL_ENABLE_NETWORKING

#include <AP_Param/AP_Param.h>
#include "lwipthread.h"
#include <AP_HAL/utility/OwnPtr.h>
#include <AP_HAL/I2CDevice.h>
#include "lwipthread.h"

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

    // lwipthread_opts_t lwip_opts;
    // uint8_t macaddress[6];

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
    //AP_HAL::OwnPtr<AP_HAL::Device> _dev;
};

namespace AP {
    AP_Networking *network();
};

#endif // HAL_ENABLE_NETWORKING
