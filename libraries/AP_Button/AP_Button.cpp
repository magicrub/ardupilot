/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include "AP_Button.h"

#if HAL_BUTTON_ENABLED

#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/GCS.h>

// very crude debounce method
#define DEBOUNCE_MS 50

extern const AP_HAL::HAL& hal;

AP_Button *AP_Button::_singleton;

const AP_Param::GroupInfo AP_Button::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable button reporting
    // @Description: This enables the button checking module. When this is disabled the parameters for setting button inputs are not visible
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 0, AP_Button, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: PIN1
    // @DisplayName: First button Pin
    // @Description: Digital pin number for first button input.  Some common values are given, but see the Wiki's "GPIOs" page for how to determine the pin number for a given autopilot.
    // @User: Standard
    // @Values: -1:Disabled,50:AUXOUT1,51:AUXOUT2,52:AUXOUT3,53:AUXOUT4,54:AUXOUT5,55:AUXOUT6
    // @Range: -1 127
    AP_GROUPINFO("PIN1",  1, AP_Button, pin[0], -1),

    // @Param: PIN2
    // @DisplayName: Second button Pin
    // @Description: Digital pin number for second button input.  Some common values are given, but see the Wiki's "GPIOs" page for how to determine the pin number for a given autopilot.
    // @User: Standard
    // @Values: -1:Disabled,50:AUXOUT1,51:AUXOUT2,52:AUXOUT3,53:AUXOUT4,54:AUXOUT5,55:AUXOUT6
    // @Range: -1 127
    AP_GROUPINFO("PIN2",  2, AP_Button, pin[1], -1),

    // @Param: PIN3
    // @DisplayName: Third button Pin
    // @Description: Digital pin number for third button input.  Some common values are given, but see the Wiki's "GPIOs" page for how to determine the pin number for a given autopilot.
    // @User: Standard
    // @Values: -1:Disabled,50:AUXOUT1,51:AUXOUT2,52:AUXOUT3,53:AUXOUT4,54:AUXOUT5,55:AUXOUT6
    // @Range: -1 127
    AP_GROUPINFO("PIN3",  3, AP_Button, pin[2], -1),

    // @Param: PIN4
    // @DisplayName: Fourth button Pin
    // @Description: Digital pin number for fourth button input. Some common values are given, but see the Wiki's "GPIOs" page for how to determine the pin number for a given autopilot.
    // @User: Standard
    // @Values: -1:Disabled,50:AUXOUT1,51:AUXOUT2,52:AUXOUT3,53:AUXOUT4,54:AUXOUT5,55:AUXOUT6
    // @Range: -1 127
    AP_GROUPINFO("PIN4",  4, AP_Button, pin[3], -1),

    // @Param: REPORT_SEND
    // @DisplayName: Report send time
    // @Description: The duration in seconds that a BUTTON_CHANGE report is repeatedly sent to the GCS regarding a button changing state. Note that the BUTTON_CHANGE message is MAVLink2 only.
    // @User: Standard
    // @Range: 0 3600
    AP_GROUPINFO("REPORT_SEND", 5, AP_Button, report_send_time, 10),

    // @Param: OPTIONS1
    // @DisplayName: Button Pin 1 Options
    // @Description: Options for Pin 1. PWM input detects PWM above or below 1800/1200us instead of logic level. If PWM is not detected or is less than 800us or above 2200us the button will interpreted as low. Invert changes HIGH state to be logic low voltage on pin, or below 1200us, if PWM input.
    // @User: Standard
    // @Bitmask: 0:PWM Input,1:InvertInput
    AP_GROUPINFO("OPTIONS1",  6, AP_Button, options[0], 0),

    // @Param: OPTIONS2
    // @DisplayName: Button Pin 2 Options
    // @Description: Options for Pin 2. PWM input detects PWM above or below 1800/1200us instead of logic level. If PWM is not detected or is less than 800us or above 2200us the button will interpreted as low. Invert changes HIGH state to be logic low voltage on pin, or below 1200us, if PWM input.
    // @User: Standard
    // @Bitmask: 0:PWM Input,1:InvertInput
    AP_GROUPINFO("OPTIONS2",  7, AP_Button, options[1], 0),

    // @Param: OPTIONS3
    // @DisplayName: Button Pin 3 Options
    // @Description: Options for Pin 3. PWM input detects PWM above or below 1800/1200us instead of logic level. If PWM is not detected or is less than 800us or above 2200us the button will interpreted as low. Invert changes HIGH state to be logic low voltage on pin, or below 1200us, if PWM input.
    // @Bitmask: 0:PWM Input,1:InvertInput
    AP_GROUPINFO("OPTIONS3",  8, AP_Button, options[2], 0),

    // @Param: OPTIONS4
    // @DisplayName: Button Pin 4 Options
    // @Description: Options for Pin 4. PWM input detects PWM above or below 1800/1200us instead of logic level. If PWM is not detected or is less than 800us or above 2200us the button will interpreted as low. Invert changes HIGH state to be logic low voltage on pin, or below 1200us, if PWM input.
    // @User: Standard
    // @Bitmask: 0:PWM Input,1:InvertInput
    AP_GROUPINFO("OPTIONS4",  9, AP_Button, options[3], 0),

    // @Param: FUNC1
    // @CopyFieldsFrom: RC1_OPTION
    // @DisplayName: Button Pin 1 RC Channel function
    // @Description: Auxiliary RC Options function executed on pin change
    // @User: Standard
    AP_GROUPINFO("FUNC1",  10, AP_Button, pin_func[0], (uint16_t)RC_Channel::AUX_FUNC::DO_NOTHING),

    // @Param: FUNC2
    // @CopyFieldsFrom: BTN_FUNC1
    // @DisplayName: Button Pin 2 RC Channel function
    AP_GROUPINFO("FUNC2",  11, AP_Button, pin_func[1], (uint16_t)RC_Channel::AUX_FUNC::DO_NOTHING),

    // @Param: FUNC3
    // @CopyFieldsFrom: BTN_FUNC1
    // @DisplayName: Button Pin 3 RC Channel function
    AP_GROUPINFO("FUNC3",  12, AP_Button, pin_func[2], (uint16_t)RC_Channel::AUX_FUNC::DO_NOTHING),

    // @Param: FUNC4
    // @CopyFieldsFrom: BTN_FUNC1
    // @DisplayName: Button Pin 4 RC Channel function
    AP_GROUPINFO("FUNC4",  13, AP_Button, pin_func[3], (uint16_t)RC_Channel::AUX_FUNC::DO_NOTHING),



#if HAL_BUTTON_MAVLINK_MANUAL_CONTROL_BUTTONS_ENABLED
    // @Param: JS1_FUNC
    // @CopyFieldsFrom: RC1_OPTION
    // @DisplayName: Joystick Button Pin 1 RC Channel function
    // @Description: Auxiliary RC Options function executed on joystick button from MANUAL_CONTROL mavlink msg
    // @User: Standard
    AP_GROUPINFO("JS1_FUNC",  20, AP_Button, manual_control_joystick_button_aux_function[0], (uint16_t)RC_Channel::AUX_FUNC::DO_NOTHING),
    AP_GROUPINFO("JS2_FUNC",  21, AP_Button, manual_control_joystick_button_aux_function[1], (uint16_t)RC_Channel::AUX_FUNC::DO_NOTHING),
    AP_GROUPINFO("JS3_FUNC",  22, AP_Button, manual_control_joystick_button_aux_function[2], (uint16_t)RC_Channel::AUX_FUNC::DO_NOTHING),
    AP_GROUPINFO("JS4_FUNC",  23, AP_Button, manual_control_joystick_button_aux_function[3], (uint16_t)RC_Channel::AUX_FUNC::DO_NOTHING),
    AP_GROUPINFO("JS5_FUNC",  24, AP_Button, manual_control_joystick_button_aux_function[4], (uint16_t)RC_Channel::AUX_FUNC::DO_NOTHING),
    AP_GROUPINFO("JS6_FUNC",  25, AP_Button, manual_control_joystick_button_aux_function[5], (uint16_t)RC_Channel::AUX_FUNC::DO_NOTHING),
    AP_GROUPINFO("JS7_FUNC",  26, AP_Button, manual_control_joystick_button_aux_function[6], (uint16_t)RC_Channel::AUX_FUNC::DO_NOTHING),
    AP_GROUPINFO("JS8_FUNC",  27, AP_Button, manual_control_joystick_button_aux_function[7], (uint16_t)RC_Channel::AUX_FUNC::DO_NOTHING),
    AP_GROUPINFO("JS9_FUNC",  28, AP_Button, manual_control_joystick_button_aux_function[8], (uint16_t)RC_Channel::AUX_FUNC::DO_NOTHING),
    AP_GROUPINFO("JS10_FUNC",  29, AP_Button, manual_control_joystick_button_aux_function[9], (uint16_t)RC_Channel::AUX_FUNC::DO_NOTHING),
    AP_GROUPINFO("JS11_FUNC",  30, AP_Button, manual_control_joystick_button_aux_function[10], (uint16_t)RC_Channel::AUX_FUNC::DO_NOTHING),
    AP_GROUPINFO("JS12_FUNC",  31, AP_Button, manual_control_joystick_button_aux_function[11], (uint16_t)RC_Channel::AUX_FUNC::DO_NOTHING),
    AP_GROUPINFO("JS13_FUNC",  32, AP_Button, manual_control_joystick_button_aux_function[12], (uint16_t)RC_Channel::AUX_FUNC::DO_NOTHING),
    AP_GROUPINFO("JS14_FUNC",  33, AP_Button, manual_control_joystick_button_aux_function[13], (uint16_t)RC_Channel::AUX_FUNC::DO_NOTHING),
    AP_GROUPINFO("JS15_FUNC",  34, AP_Button, manual_control_joystick_button_aux_function[14], (uint16_t)RC_Channel::AUX_FUNC::DO_NOTHING),
    AP_GROUPINFO("JS16_FUNC",  35, AP_Button, manual_control_joystick_button_aux_function[15], (uint16_t)RC_Channel::AUX_FUNC::DO_NOTHING),
    AP_GROUPINFO("JS17_FUNC",  36, AP_Button, manual_control_joystick_button_aux_function[16], (uint16_t)RC_Channel::AUX_FUNC::DO_NOTHING),
    AP_GROUPINFO("JS18_FUNC",  37, AP_Button, manual_control_joystick_button_aux_function[17], (uint16_t)RC_Channel::AUX_FUNC::DO_NOTHING),
    AP_GROUPINFO("JS19_FUNC",  38, AP_Button, manual_control_joystick_button_aux_function[18], (uint16_t)RC_Channel::AUX_FUNC::DO_NOTHING),
    AP_GROUPINFO("JS20_FUNC",  39, AP_Button, manual_control_joystick_button_aux_function[19], (uint16_t)RC_Channel::AUX_FUNC::DO_NOTHING),
    AP_GROUPINFO("JS21_FUNC",  40, AP_Button, manual_control_joystick_button_aux_function[20], (uint16_t)RC_Channel::AUX_FUNC::DO_NOTHING),
    AP_GROUPINFO("JS22_FUNC",  41, AP_Button, manual_control_joystick_button_aux_function[21], (uint16_t)RC_Channel::AUX_FUNC::DO_NOTHING),
    AP_GROUPINFO("JS23_FUNC",  42, AP_Button, manual_control_joystick_button_aux_function[22], (uint16_t)RC_Channel::AUX_FUNC::DO_NOTHING),
    AP_GROUPINFO("JS24_FUNC",  43, AP_Button, manual_control_joystick_button_aux_function[23], (uint16_t)RC_Channel::AUX_FUNC::DO_NOTHING),
    AP_GROUPINFO("JS25_FUNC",  44, AP_Button, manual_control_joystick_button_aux_function[24], (uint16_t)RC_Channel::AUX_FUNC::DO_NOTHING),
    AP_GROUPINFO("JS26_FUNC",  45, AP_Button, manual_control_joystick_button_aux_function[25], (uint16_t)RC_Channel::AUX_FUNC::DO_NOTHING),
    AP_GROUPINFO("JS27_FUNC",  46, AP_Button, manual_control_joystick_button_aux_function[26], (uint16_t)RC_Channel::AUX_FUNC::DO_NOTHING),
    AP_GROUPINFO("JS28_FUNC",  47, AP_Button, manual_control_joystick_button_aux_function[27], (uint16_t)RC_Channel::AUX_FUNC::DO_NOTHING),
    AP_GROUPINFO("JS29_FUNC",  48, AP_Button, manual_control_joystick_button_aux_function[28], (uint16_t)RC_Channel::AUX_FUNC::DO_NOTHING),
    AP_GROUPINFO("JS30_FUNC",  49, AP_Button, manual_control_joystick_button_aux_function[29], (uint16_t)RC_Channel::AUX_FUNC::DO_NOTHING),
    AP_GROUPINFO("JS31_FUNC",  50, AP_Button, manual_control_joystick_button_aux_function[30], (uint16_t)RC_Channel::AUX_FUNC::DO_NOTHING),
    AP_GROUPINFO("JS32_FUNC",  51, AP_Button, manual_control_joystick_button_aux_function[31], (uint16_t)RC_Channel::AUX_FUNC::DO_NOTHING),

    AP_GROUPINFO("JS_INIT_BM",  52, AP_Button, manual_control_joystick_button_init_at_boot_bitmask, 0),
#endif // HAL_BUTTON_MAVLINK_MANUAL_CONTROL_BUTTONS_ENABLED

    AP_GROUPEND    
};


// constructor
AP_Button::AP_Button(void)
{
    AP_Param::setup_object_defaults(this, var_info);

    if (_singleton != nullptr) {
        AP_HAL::panic("AP_Button must be singleton");
    }
    _singleton = this;
}

/*
  update and report, called from main loop
 */
void AP_Button::update(void)
{
    if (!enable) {
        return;
    }

    // call setup pins at update rate (5Hz) to allow for runtime parameter change of pins
    setup_pins();

    if (!initialised) {
        initialised = true;

#if HAL_BUTTON_MAVLINK_MANUAL_CONTROL_BUTTONS_ENABLED
        if (manual_control_joystick_button_init_at_boot_bitmask.get() != 0) {
             // force all buttons to be seen as changed at first call
            button_state_prev = manual_control_joystick_button_init_at_boot_bitmask.get(); // set these bits as 1 so they detect a change
            handle_manual_control_buttons(0,0);
        }
#endif // HAL_BUTTON_MAVLINK_MANUAL_CONTROL_BUTTONS_ENABLED

        // get initial mask
        last_mask = get_mask();
        debounce_mask = last_mask;

        // register 1kHz timer callback
        hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_Button::timer_update, void));        
    }

    // act on any changes in state
    {
        WITH_SEMAPHORE(last_debounced_change_ms_sem);
        if (last_debounced_change_ms > last_debounce_ms) {
            last_debounce_ms = last_debounced_change_ms;
        }
    }

    // update the PWM state:
    uint8_t new_pwm_state = pwm_state;
    for (uint8_t i=0; i<AP_BUTTON_NUM_PINS; i++) {
        const uint8_t mask = (1U << i);
        if (!is_pwm_input(i)) {
            // not a PWM input
            new_pwm_state &= ~mask;
            continue;
        }
        const uint16_t pwm_us = pwm_pin_source[i].get_pwm_us();
        if (pwm_us < RC_Channel::RC_MIN_LIMIT_PWM || pwm_us > RC_Channel::RC_MAX_LIMIT_PWM) {
            // invalid pulse width, trigger low
            if (pwm_state & mask) {
                new_pwm_state &= ~mask;
            }
            continue;
        }
        // these values are the same as used in RC_Channel:
        if (pwm_state & mask) {
            // currently asserted; check to see if we should de-assert
            if (pwm_us < RC_Channel::AUX_SWITCH_PWM_TRIGGER_LOW) {
                new_pwm_state &= ~mask;
            }
        } else {
            // currently not asserted; check to see if we should assert
            if (pwm_us > RC_Channel::AUX_SWITCH_PWM_TRIGGER_HIGH) {
                new_pwm_state |= mask;
            }
        }
    }
    const uint64_t now_ms = AP_HAL::millis64();
    if (new_pwm_state != pwm_state) {
        if (new_pwm_state != tentative_pwm_state) {
            tentative_pwm_state = new_pwm_state;
            pwm_start_debounce_ms = now_ms;
        } else if (now_ms - pwm_start_debounce_ms > DEBOUNCE_MS) {
            pwm_state = new_pwm_state;
            last_debounce_ms = now_ms;
        }
    } else {
        tentative_pwm_state = pwm_state;
        pwm_start_debounce_ms = now_ms;
    }

#if HAL_GCS_ENABLED
    if (last_debounce_ms != 0 &&
        (AP_HAL::millis() - last_report_ms) > AP_BUTTON_REPORT_PERIOD_MS &&
        (AP_HAL::millis64() - last_debounce_ms) < report_send_time*1000ULL) {
        // send a change report
        last_report_ms = AP_HAL::millis();

        // send a report to GCS
        send_report();
    }
#endif

    if (!aux_functions_initialised) {
        run_aux_functions(true);
        aux_functions_initialised = true;
    }

    if (last_debounce_ms != 0 &&
        last_debounce_ms != last_action_time_ms) {
        last_action_time_ms = last_debounce_ms;
        run_aux_functions(false);
    }
}

void AP_Button::run_aux_functions(bool force)
{
    RC_Channel *rc_channel = rc().channel(1);
    if (rc_channel == nullptr) {
        return;
    }

    for (uint8_t i=0; i<AP_BUTTON_NUM_PINS; i++) {
        const RC_Channel::AUX_FUNC func = RC_Channel::AUX_FUNC(pin_func[i].get());
        if (func == RC_Channel::AUX_FUNC::DO_NOTHING) {
            continue;
        }
        const uint8_t value_mask = (1U<<i);
        bool value;
        if (is_pwm_input(i)) {
            value = (pwm_state & value_mask) != 0;
        } else {
            value = (debounce_mask & value_mask) != 0;
        }
        if (is_input_inverted(i)) {
            value = !value;
        }
        const bool actioned = ((state_actioned_mask & value_mask) != 0);
        if (!force && value == actioned) {
            // no change on this pin
            continue;
        }
        // mark action as done:
        if (value) {
            state_actioned_mask |= value_mask;
        } else {
            state_actioned_mask &= ~value_mask;
        }

        const RC_Channel::AuxSwitchPos pos = value ? RC_Channel::AuxSwitchPos::HIGH : RC_Channel::AuxSwitchPos::LOW;
        // I wonder if we can do better here:
#if AP_RC_CHANNEL_AUX_FUNCTION_STRINGS_ENABLED
        const char *str = rc_channel->string_for_aux_function(func);
        if (str != nullptr) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Button %i: executing (%s %s)", i+1, str, rc_channel->string_for_aux_pos(pos));
        }
#endif
        rc_channel->run_aux_function(func, pos, RC_Channel::AuxFuncTrigger::Source::BUTTON, i);
    }
}

// get state of a button
// used by scripting
bool AP_Button::get_button_state(uint8_t number)
{
    // pins params are 1 indexed not zero
    if (number == 0 || number > AP_BUTTON_NUM_PINS) {
        return false;
    }

    if (is_pwm_input(number-1)) {
        return (pwm_state & (1U<<(number-1)));
    }

    return ( ((1 << (number - 1)) & debounce_mask) != 0);
};

/*
  get current mask
 */
uint8_t AP_Button::get_mask(void)
{
    uint8_t mask = 0;
    for (uint8_t i=0; i<AP_BUTTON_NUM_PINS; i++) {
        if (pin[i] == -1) {
            continue;
        }
        if (is_pwm_input(i)) {
            continue;
        }
        mask |= hal.gpio->read(pin[i]) << i;
    }

    return mask;
}

/*
  called at 1kHz to check for button state change
 */
void AP_Button::timer_update(void)
{
    if (!enable) {
        return;
    }
    uint8_t mask = get_mask();
    uint64_t now = AP_HAL::millis64();
    if (mask != last_mask) {
        last_mask = mask;
        last_change_time_ms = now;
    }
    if (debounce_mask != last_mask &&
        (now - last_change_time_ms) > DEBOUNCE_MS) {
        // crude de-bouncing, debounces all buttons as one, not individually
        debounce_mask = last_mask;
        WITH_SEMAPHORE(last_debounced_change_ms_sem);
        last_debounced_change_ms = now;
    }
}

#if HAL_GCS_ENABLED
/*
  send a BUTTON_CHANGE report to the GCS
 */
void AP_Button::send_report(void) const
{
    const uint8_t mask = last_mask | pwm_state;
    const mavlink_button_change_t packet{
            time_boot_ms: AP_HAL::millis(),
            last_change_ms: uint32_t(last_debounce_ms),
            state: mask,
    };
    gcs().send_to_active_channels(MAVLINK_MSG_ID_BUTTON_CHANGE,
                                  (const char *)&packet);
}
#endif

/*
  setup the pins as input with pullup. We need pullup to give reliable
  input with a pulldown button
 */
void AP_Button::setup_pins(void)
{
    for (uint8_t i=0; i<AP_BUTTON_NUM_PINS; i++) {
        if (is_pwm_input(i)) {
            pwm_pin_source[i].set_pin(pin[i], "Button");
            continue;
        }
        if (pin[i] == -1) {
            continue;
        }

        hal.gpio->pinMode(pin[i], HAL_GPIO_INPUT);
        // setup pullup
        hal.gpio->write(pin[i], 1);
    }
}

// check settings are valid
bool AP_Button::arming_checks(size_t buflen, char *buffer) const
{
    if (!enable) {
        return true;
    }
    for (uint8_t i=0; i<AP_BUTTON_NUM_PINS; i++) {
        if (pin[i] != -1 && !hal.gpio->valid_pin(pin[i])) {
            uint8_t servo_ch;
            if (hal.gpio->pin_to_servo_channel(pin[i], servo_ch)) {
                hal.util->snprintf(buffer, buflen, "BTN_PIN%u=%d, set SERVO%u_FUNCTION=-1", unsigned(i + 1), int(pin[i].get()), unsigned(servo_ch+1));
            } else {
                hal.util->snprintf(buffer, buflen, "BTN_PIN%u=%d invalid", unsigned(i + 1), int(pin[i].get()));
            }
            return false;
        }
    }
    return true;
}

#if HAL_BUTTON_MAVLINK_MANUAL_CONTROL_BUTTONS_ENABLED
void AP_Button::handle_manual_control_buttons(const uint16_t buttons, const uint16_t buttons2)
{
    if (!enable) {
        return;
    }

    const uint32_t button_state = (uint32_t(buttons2) << 16) | uint32_t(buttons);
    if (button_state == button_state_prev) {
        // no change on any buttons
        return;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "Got Button msg with a button change");

    for (uint8_t i=0; i<ARRAY_SIZE(manual_control_joystick_button_aux_function); i++) {
        const uint32_t mask = (1U << i);
        if ((button_state & mask) == (button_state_prev & mask)) {
            // no change on this button
            continue;
        }
        
        const RC_Channel::AUX_FUNC aux_func = (RC_Channel::AUX_FUNC)manual_control_joystick_button_aux_function[i].get();
        const RC_Channel::AuxSwitchPos position = ((button_state & mask) != 0) ? RC_Channel::AuxSwitchPos::HIGH : RC_Channel::AuxSwitchPos::LOW;
        if (aux_func == RC_Channel::AUX_FUNC::DO_NOTHING) {
            // no function configured, nothing to do

            GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "Btn_JS %u, Pos %u, Func NONE",
                      unsigned(i+1),
                      (unsigned)position);
            continue;
        }

        rc().run_aux_function(aux_func, position, RC_Channel::AuxFuncTrigger::Source::MAVLINK, i);

        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "Btn_JS %u, Pos %u, Func %u",
                      unsigned(i+1),
                      (unsigned)position,
                      (unsigned)aux_func);

    }

    // store previous state. This must be done at the end so we know what changed
    button_state_prev = button_state;
}
#endif // HAL_BUTTON_MAVLINK_MANUAL_CONTROL_BUTTONS_ENABLED

namespace AP {

AP_Button &button()
{
    return *AP_Button::get_singleton();
}

}

#endif
