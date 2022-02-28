
#pragma once

#include <AP_HAL/AP_HAL.h>
#if HAL_ENABLE_NETWORKING

#if defined(BOARD_PHY_USE_CACHED_LINK_STATUS) && !defined(AP_NETWORKING_PHY_COUNT)
#define AP_NETWORKING_PHY_COUNT 1
#endif

#ifndef AP_NETWORKING_PHY_PRIMARY
#define AP_NETWORKING_PHY_PRIMARY 1
#endif


#include <AP_Param/AP_Param.h>
#include "lwipthread.h"
#include <AP_HAL/utility/OwnPtr.h>
#include <AP_HAL/I2CDevice.h>

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

private:
    static AP_Networking *_singleton;
    void thread();
    void reset();
    void configure();
    uint8_t read_register8(const uint8_t port, const uint8_t function, const uint8_t reg);
    uint16_t read_register16(const uint8_t port, const uint8_t function, const uint8_t reg);
    bool write_register8(const uint8_t port, const uint8_t function, const uint8_t reg, const uint8_t data);
    bool write_register16(const uint8_t port, const uint8_t function, const uint8_t reg, const uint16_t data);

    uint16_t convert_to_addr16(const uint8_t port, const uint8_t function, const uint8_t reg);

    struct {
        //bool done;
        uint32_t timer_ms;
        uint8_t reset_state;
        bool reset_done;
        bool is_configured;
        bool failed;
    } _init;

    struct {
        uint32_t timer_ms;
        uint32_t state;
    } _debug;

    struct {
        AP_Int16 ipaddr[4];
        AP_Int16 netmask;
        AP_Int16 dhcp;
        AP_Int16 gwaddr[4];
        AP_Int16 macaddr[6];
    } _param;

#ifdef BOARD_PHY_USE_CACHED_LINK_STATUS
    struct {
        uint32_t timer_ms;
        struct {
            uint32_t bmsr;
            uint32_t bmcr;
            uint32_t lpa;
        } registers;
    } _link_stats[AP_NETWORKING_PHY_COUNT];
#endif

    lwipthread_opts_t lwip_opts;
    uint8_t macaddress[6];

    HAL_Semaphore _sem;
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
};

namespace AP {
    AP_Networking *network();
};

#endif // HAL_ENABLE_NETWORKING
