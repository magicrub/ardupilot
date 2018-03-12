#include "AP_LandingGear.h"
#include <AP_Relay/AP_Relay.h>
#include <AP_Math/AP_Math.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_LandingGear::var_info[] = {

    // @Param: SERVO_RTRACT
    // @DisplayName: Landing Gear Servo Retracted PWM Value
    // @Description: Servo PWM value in microseconds when landing gear is retracted
    // @Range: 1000 2000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SERVO_RTRACT", 0, AP_LandingGear, _servo_retract_pwm, AP_LANDINGGEAR_SERVO_RETRACT_PWM_DEFAULT),

    // @Param: SERVO_DEPLOY
    // @DisplayName: Landing Gear Servo Deployed PWM Value
    // @Description: Servo PWM value in microseconds when landing gear is deployed
    // @Range: 1000 2000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SERVO_DEPLOY", 1, AP_LandingGear, _servo_deploy_pwm, AP_LANDINGGEAR_SERVO_DEPLOY_PWM_DEFAULT),

    // @Param: STARTUP
    // @DisplayName: Landing Gear Startup position
    // @Description: Landing Gear Startup behaviour control
    // @Values: 0:WaitForPilotInput, 1:Retract, 2:Deploy
    // @User: Standard
    AP_GROUPINFO("STARTUP", 2, AP_LandingGear, _startup_behaviour, (uint8_t)AP_LandingGear::LandingGear_Startup_WaitForPilotInput),
    
    // @Param: DEPLOY_PIN
    // @DisplayName: Chassis deployment feedback pin
    // @Description: Pin number to use for feedback of gear deployment. If set to -1 feedback is disabled.
    // @Values: -1:Disabled,50:PX4 AUX1,51:PX4 AUX2,52:PX4 AUX3,53:PX4 AUX4,54:PX4 AUX5,55:PX4 AUX6
    // @User: Standard
    AP_GROUPINFO("DEPLOY_PIN", 3, AP_LandingGear, _pin_deployed, -1),

    // @Param: DEPLOY_POL
    // @DisplayName: Chassis deployment feedback pin polarity
    // @Description: Polarity for feedback pin. If this is 1 then the pin should be high when gear are deployed. If set to 0 then then deployed gear level is low.
    // @Values: 0:Low,1:High
    // @User: Standard
    AP_GROUPINFO("DEPLOY_POL", 4, AP_LandingGear, _pin_deployed_polarity, 0),
    
    // @Param: WOW_PIN
    // @DisplayName: Weight on wheels feedback pin
    // @Description: Pin number to use for feedback of weight on wheels condition. If set to -1 feedback is disabled.
    // @Values: -1:Disabled,50:PX4 AUX1,51:PX4 AUX2,52:PX4 AUX3,53:PX4 AUX4,54:PX4 AUX5,55:PX4 AUX6
    // @User: Standard
    AP_GROUPINFO("WOW_PIN", 5, AP_LandingGear, _pin_weight_on_wheels, DEFAULT_PIN_WOW),

    // @Param: WOW_POL
    // @DisplayName: Weight on wheels feedback pin polarity
    // @Description: Polarity for feedback pin. If this is 1 then the pin should be high when there is weight on wheels. If set to 0 then then weight on wheels level is low.
    // @Values: 0:Low,1:High
    // @User: Standard
    AP_GROUPINFO("WOW_POL", 6, AP_LandingGear, _pin_weight_on_wheels_polarity, DEFAULT_PIN_WOW_POL),
    
    AP_GROUPEND
};

AP_LandingGear *AP_LandingGear::_instance;

/// initialise state of landing gear
void AP_LandingGear::init()
{
    if (_pin_deployed != -1) {
        hal.gpio->pinMode(_pin_deployed, HAL_GPIO_INPUT);
        hal.gpio->write(_pin_deployed, 1);
    }
    
    if (_pin_weight_on_wheels != -1) {
        hal.gpio->pinMode(_pin_weight_on_wheels, HAL_GPIO_INPUT);
        hal.gpio->write(_pin_weight_on_wheels, 1);
    }
    
    switch ((enum LandingGearStartupBehaviour)_startup_behaviour.get()) {
        default:
        case LandingGear_Startup_WaitForPilotInput:
            // do nothing
            break;
        case LandingGear_Startup_Retract:
            retract();
            break;
        case LandingGear_Startup_Deploy:
            deploy();
            break;
    }
}

/// set landing gear position to retract, deploy or deploy-and-keep-deployed
void AP_LandingGear::set_position(LandingGearCommand cmd)
{
    switch (cmd) {
        case LandingGear_Retract:
            if (!_deploy_lock) {
                retract();
            }
            break;
        case LandingGear_Deploy:
            deploy();
            _deploy_lock = false;
            break;
        case LandingGear_Deploy_And_Keep_Deployed:
            deploy();
            _deploy_lock = true;
            break;
    }
}

/// deploy - deploy landing gear
void AP_LandingGear::deploy()
{
    // set servo PWM to deployed position
    SRV_Channels::set_output_pwm(SRV_Channel::k_landing_gear_control, _servo_deploy_pwm);

    // set deployed flag
    _deployed = true;
}

/// retract - retract landing gear
void AP_LandingGear::retract()
{
    // set servo PWM to retracted position
    SRV_Channels::set_output_pwm(SRV_Channel::k_landing_gear_control, _servo_retract_pwm);

    // reset deployed flag
    _deployed = false;
}

bool AP_LandingGear::deployed()
{
    if (_pin_deployed == -1) {
        return _deployed;
    } else {
        return hal.gpio->read(_pin_deployed) == _pin_deployed_polarity ? true : false;
    }
}

AP_LandingGear::LG_WOW_State AP_LandingGear::wow()
{
    if (_pin_weight_on_wheels == -1) {
        return LG_WOW_UNKNOWN;
    } else {
        return hal.gpio->read(_pin_weight_on_wheels) == _pin_weight_on_wheels_polarity ? LG_WOW : LG_NO_WOW;
    }
}

AP_LandingGear::LG_LandingGear_State AP_LandingGear::get_state()
{
    if (_pin_deployed == -1) {
        if (_deployed) {
            return LG_DEPLOYED;
        } else {
            return LG_RETRACTED;
        }
    } else {
        if (_deployed && deployed()) {
            return LG_DEPLOYED;
        }
        if (!_deployed && !deployed()) {
            return LG_RETRACTED;
        }
        if (_deployed && !deployed()) {
            return LG_DEPLOYING;
        }
        if (!_deployed && deployed()) {
            return LG_RETRACTING;
        }
        
        return LG_UNKNOWN;
    }
}