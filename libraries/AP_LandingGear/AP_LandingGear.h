/// @file	AP_LandingGear.h
/// @brief	Landing gear control library
#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Common/AP_Common.h>

#define AP_LANDINGGEAR_SERVO_RETRACT_PWM_DEFAULT    1250    // default PWM value to move servo to when landing gear is up
#define AP_LANDINGGEAR_SERVO_DEPLOY_PWM_DEFAULT     1750    // default PWM value to move servo to when landing gear is down

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#define DEFAULT_PIN_WOW 8
#define DEFAULT_PIN_WOW_POL 1
#else
#define DEFAULT_PIN_WOW -1
#define DEFAULT_PIN_WOW_POL 0
#endif

/// @class	AP_LandingGear
/// @brief	Class managing the control of landing gear
class AP_LandingGear {
public:
    AP_LandingGear() {
        // setup parameter defaults
        AP_Param::setup_object_defaults(this, var_info);
        
        if (_instance != nullptr) {
            AP_HAL::panic("AP_LandingGear must be singleton");
        }
        _instance = this;
    }

    /* Do not allow copies */
    AP_LandingGear(const AP_LandingGear &other) = delete;
    AP_LandingGear &operator=(const AP_LandingGear&) = delete;
    
    // get singleton instance
    static AP_LandingGear &instance(void) {
        return *_instance;
    }

    // Gear command modes
    enum LandingGearCommand {
        LandingGear_Retract,
        LandingGear_Deploy,
        LandingGear_Deploy_And_Keep_Deployed,
    };

    // Gear command modes
    enum LandingGearStartupBehaviour {
        LandingGear_Startup_WaitForPilotInput = 0,
        LandingGear_Startup_Retract = 1,
        LandingGear_Startup_Deploy = 2,
    };

    /// initialise state of landing gear
    void init();

    /// returns true if the landing gear is deployed
    bool deployed();
    
    enum LG_LandingGear_State {
        LG_UNKNOWN = -1,
        LG_RETRACTED = 0,
        LG_DEPLOYED = 1,
        LG_RETRACTING = 2,
        LG_DEPLOYING = 3,
        };
    /// returns detailed state of gear
    LG_LandingGear_State get_state();
    
    enum LG_WOW_State {
        LG_WOW_UNKNOWN = -1,
        LG_NO_WOW = 0,
        LG_WOW = 1,
        };

    LG_WOW_State get_wow_state();

    /// set landing gear position to retract, deploy or deploy-and-keep-deployed
    void set_position(LandingGearCommand cmd);

    static const struct AP_Param::GroupInfo        var_info[];

private:
    // Parameters
    AP_Int16    _servo_retract_pwm;     // PWM value to move servo to when gear is retracted
    AP_Int16    _servo_deploy_pwm;      // PWM value to move servo to when gear is deployed
    AP_Int8     _startup_behaviour;     // start-up behaviour (see LandingGearStartupBehaviour)
    
    AP_Int8     _pin_deployed;
    AP_Int8     _pin_deployed_polarity;
    AP_Int8     _pin_weight_on_wheels;
    AP_Int8     _pin_weight_on_wheels_polarity;

    // internal variables
    bool        _deployed;              // true if the landing gear has been deployed, initialized false
    bool        _deploy_lock;           // used to force landing gear to remain deployed until another regular Deploy command is received to reduce accidental retraction
    
    /// retract - retract landing gear
    void retract();
    
    /// deploy - deploy the landing gear
    void deploy();
    
    static AP_LandingGear *_instance;
};
