/*
 * AP_Relay.h
 *
 *  Created on: Oct 2, 2011
 *      Author: Amilcar Lucas
 */

/// @file	AP_Relay.h
/// @brief	APM relay control class
#pragma once

#include <AP_Param/AP_Param.h>

#ifndef AP_RELAY_NUM_RELAYS
#define AP_RELAY_NUM_RELAYS 6
#endif

/// @class	AP_Relay
/// @brief	Class to manage the ArduPilot relay
class AP_Relay {
public:
    AP_Relay();

    /* Do not allow copies */
    AP_Relay(const AP_Relay &other) = delete;
    AP_Relay &operator=(const AP_Relay&) = delete;

    // setup the relay pin
    void        init();

    // activate the relay
    void        on(uint8_t instance) { set(instance, true); }

    // de-activate the relay
    void        off(uint8_t instance) { set(instance, false); }

    // see if the relay is enabled
    bool        enabled(uint8_t instance) const { return instance < AP_RELAY_NUM_RELAYS && _pin[instance] != -1; }

    // see if ANY relay is enabled
    bool        enabled() const { for (uint8_t i=0; i<AP_RELAY_NUM_RELAYS; i++) { if (enabled(i)) { return true; } } return false; }

    // get a bitfield of the state of all the pins
    uint32_t get_pin_values() const { return _pin_values; }

    // activate the relay with logic level value.
    void        set(uint8_t instance, bool value);

    // toggle the relay status
    void        toggle(uint8_t instance);

    // check settings are valid
    bool arming_checks(size_t buflen, char *buffer) const;
    
    static AP_Relay *get_singleton(void) {return singleton; }

    static const struct AP_Param::GroupInfo        var_info[];

private:
    static AP_Relay *singleton;

    AP_Int8 _pin[AP_RELAY_NUM_RELAYS];
    AP_Int8 _default;

    uint32_t _pin_values;
};

namespace AP {
    AP_Relay *relay();
};
