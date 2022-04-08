#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Common/Location.h>
#include <stdint.h>
#include <AP_Common/Location.h>
#include <AP_Soaring/AP_Soaring.h>
#include <AP_ADSB/AP_ADSB.h>
#include <AP_Vehicle/ModeReason.h>

#ifndef HAL_STALL_RECOVERY_ENABLED
#define HAL_STALL_RECOVERY_ENABLED !HAL_MINIMIZE_FEATURES && BOARD_FLASH_SIZE > 1024
#endif

class Mode
{
public:

    /* Do not allow copies */
    Mode(const Mode &other) = delete;
    Mode &operator=(const Mode&) = delete;

    // Auto Pilot modes
    // ----------------
    enum Number : uint8_t {
        MANUAL        = 0,
        CIRCLE        = 1,
        STABILIZE     = 2,
        TRAINING      = 3,
        ACRO          = 4,
        FLY_BY_WIRE_A = 5,
        FLY_BY_WIRE_B = 6,
        CRUISE        = 7,
        AUTOTUNE      = 8,
        AUTO          = 10,
        RTL           = 11,
        LOITER        = 12,
        TAKEOFF       = 13,
        AVOID_ADSB    = 14,
        GUIDED        = 15,
        INITIALISING  = 16,
        QSTABILIZE    = 17,
        QHOVER        = 18,
        QLOITER       = 19,
        QLAND         = 20,
        QRTL          = 21,
        QAUTOTUNE     = 22,
        QACRO         = 23,
        THERMAL       = 24,
        STALLRECOVERY = 25,
    };

    // Constructor
    Mode();

    // enter this mode, always returns true/success
    bool enter();

    // perform any cleanups required:
    void exit();

    // returns a unique number specific to this mode
    virtual Number mode_number() const = 0;

    // returns full text name
    virtual const char *name() const = 0;

    // returns a string for this flightmode, exactly 4 bytes
    virtual const char *name4() const = 0;

    // returns true if the vehicle can be armed in this mode
    virtual bool allows_arming() const { return true; }

    //
    // methods that sub classes should override to affect movement of the vehicle in this mode
    //

    // convert user input to targets, implement high level control for this mode
    virtual void update() = 0;

    // true for all q modes
    virtual bool is_vtol_mode() const { return false; }
    virtual bool is_vtol_man_throttle() const;
    virtual bool is_vtol_man_mode() const { return false; }
    // guided or adsb mode
    virtual bool is_guided_mode() const { return false; }

    // true if mode can have terrain following disabled by switch
    virtual bool allows_terrain_disable() const { return false; }

    // true if automatic switch to thermal mode is supported.
    virtual bool does_automatic_thermal_switch() const {return false; }

    // subclasses override this if they require navigation.
    virtual void navigate() { return; }

    // this allows certain flight modes to mix RC input with throttle
    // depending on airspeed_nudge_cm
    virtual bool allows_throttle_nudging() const { return false; }

    // true if the mode sets the vehicle destination, which controls
    // whether control input is ignored with STICK_MIXING=0
    virtual bool does_auto_navigation() const { return false; }

    // true if the mode sets the vehicle destination, which controls
    // whether control input is ignored with STICK_MIXING=0
    virtual bool does_auto_throttle() const { return false; }

    // method for mode specific target altitude profiles
    virtual bool update_target_altitude() { return false; }
    
protected:

    // subclasses override this to perform checks before entering the mode
    virtual bool _enter() { return true; }

    // subclasses override this to perform any required cleanup when exiting the mode
    virtual void _exit() { return; }
};


class ModeAcro : public Mode
{
public:

    Mode::Number mode_number() const override { return Mode::Number::ACRO; }
    const char *name() const override { return "ACRO"; }
    const char *name4() const override { return "ACRO"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
};

class ModeAuto : public Mode
{
public:

    Number mode_number() const override { return Number::AUTO; }
    const char *name() const override { return "AUTO"; }
    const char *name4() const override { return "AUTO"; }

    bool does_automatic_thermal_switch() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_navigation() const override { return true; }

    bool does_auto_throttle() const override { return true; }

protected:

    bool _enter() override;
    void _exit() override;
};


class ModeAutoTune : public Mode
{
public:

    Number mode_number() const override { return Number::AUTOTUNE; }
    const char *name() const override { return "AUTOTUNE"; }
    const char *name4() const override { return "ATUN"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
    void _exit() override;
};

class ModeGuided : public Mode
{
public:

    Number mode_number() const override { return Number::GUIDED; }
    const char *name() const override { return "GUIDED"; }
    const char *name4() const override { return "GUID"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    virtual bool is_guided_mode() const override { return true; }

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_navigation() const override { return true; }

    bool does_auto_throttle() const override { return true; }

    // adjustable radius for guided. A radius of Zero indicates to use WP_LOITER_RAD
    uint16_t active_radius_m;

protected:

    bool _enter() override;
};

class ModeCircle: public Mode
{
public:

    Number mode_number() const override { return Number::CIRCLE; }
    const char *name() const override { return "CIRCLE"; }
    const char *name4() const override { return "CIRC"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    bool does_auto_navigation() const override { return true; }

    bool does_auto_throttle() const override { return true; }

protected:

    bool _enter() override;
};

class ModeLoiter : public Mode
{
public:

    Number mode_number() const override { return Number::LOITER; }
    const char *name() const override { return "LOITER"; }
    const char *name4() const override { return "LOIT"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    bool isHeadingLinedUp(const Location loiterCenterLoc, const Location targetLoc);
    bool isHeadingLinedUp_cd(const int32_t bearing_cd);

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_navigation() const override { return true; }

    bool does_auto_throttle() const override { return true; }

protected:

    bool _enter() override;
};

class ModeManual : public Mode
{
public:

    Number mode_number() const override { return Number::MANUAL; }
    const char *name() const override { return "MANUAL"; }
    const char *name4() const override { return "MANU"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    void _exit() override;
};

class ModeRTL : public Mode
{
public:

    Number mode_number() const override { return Number::RTL; }
    const char *name() const override { return "RTL"; }
    const char *name4() const override { return "RTL "; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_navigation() const override { return true; }

    bool does_auto_throttle() const override { return true; }

protected:

    bool _enter() override;

private:

    // Switch to QRTL if enabled and within radius
    bool switch_QRTL(bool check_loiter_target = true);
};

class ModeStabilize : public Mode
{
public:

    Number mode_number() const override { return Number::STABILIZE; }
    const char *name() const override { return "STABILIZE"; }
    const char *name4() const override { return "STAB"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;
};

class ModeTraining : public Mode
{
public:

    Number mode_number() const override { return Number::TRAINING; }
    const char *name() const override { return "TRAINING"; }
    const char *name4() const override { return "TRAN"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;
};

class ModeInitializing : public Mode
{
public:

    Number mode_number() const override { return Number::INITIALISING; }
    const char *name() const override { return "INITIALISING"; }
    const char *name4() const override { return "INIT"; }

    bool _enter() override { return false; }

    // methods that affect movement of the vehicle in this mode
    void update() override { }

    bool allows_arming() const override { return false; }

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_throttle() const override { return true; }
};

class ModeFBWA : public Mode
{
public:

    Number mode_number() const override { return Number::FLY_BY_WIRE_A; }
    const char *name() const override { return "FLY_BY_WIRE_A"; }
    const char *name4() const override { return "FBWA"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

};

class ModeFBWB : public Mode
{
public:

    Number mode_number() const override { return Number::FLY_BY_WIRE_B; }
    const char *name() const override { return "FLY_BY_WIRE_B"; }
    const char *name4() const override { return "FBWB"; }

    bool allows_terrain_disable() const override { return true; }

    bool does_automatic_thermal_switch() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    bool does_auto_throttle() const override { return true; }

protected:

    bool _enter() override;
};

class ModeCruise : public Mode
{
public:

    Number mode_number() const override { return Number::CRUISE; }
    const char *name() const override { return "CRUISE"; }
    const char *name4() const override { return "CRUS"; }

    bool allows_terrain_disable() const override { return true; }

    bool does_automatic_thermal_switch() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    bool get_target_heading_cd(int32_t &target_heading) const;

    bool does_auto_throttle() const override { return true; }

protected:

    bool _enter() override;

    bool locked_heading;
    int32_t locked_heading_cd;
    uint32_t lock_timer_ms;
};

#if HAL_ADSB_ENABLED
class ModeAvoidADSB : public Mode
{
public:

    Number mode_number() const override { return Number::AVOID_ADSB; }
    const char *name() const override { return "AVOID_ADSB"; }
    const char *name4() const override { return "AVOI"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    virtual bool is_guided_mode() const override { return true; }

    bool does_auto_throttle() const override { return true; }

protected:

    bool _enter() override;
};
#endif

class ModeQStabilize : public Mode
{
public:

    Number mode_number() const override { return Number::QSTABILIZE; }
    const char *name() const override { return "QSTABILIZE"; }
    const char *name4() const override { return "QSTB"; }

    bool is_vtol_mode() const override { return true; }
    bool is_vtol_man_throttle() const override { return true; }
    virtual bool is_vtol_man_mode() const override { return true; }
    bool allows_throttle_nudging() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // used as a base class for all Q modes
    bool _enter() override;

protected:
private:

    void set_tailsitter_roll_pitch(const float roll_input, const float pitch_input);
    void set_limited_roll_pitch(const float roll_input, const float pitch_input);

};

class ModeQHover : public Mode
{
public:

    Number mode_number() const override { return Number::QHOVER; }
    const char *name() const override { return "QHOVER"; }
    const char *name4() const override { return "QHOV"; }

    bool is_vtol_mode() const override { return true; }
    virtual bool is_vtol_man_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
};

class ModeQLoiter : public Mode
{
public:

    Number mode_number() const override { return Number::QLOITER; }
    const char *name() const override { return "QLOITER"; }
    const char *name4() const override { return "QLOT"; }

    bool is_vtol_mode() const override { return true; }
    virtual bool is_vtol_man_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
};

class ModeQLand : public Mode
{
public:

    Number mode_number() const override { return Number::QLAND; }
    const char *name() const override { return "QLAND"; }
    const char *name4() const override { return "QLND"; }

    bool is_vtol_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    bool allows_arming() const override { return false; }

protected:

    bool _enter() override;
};

class ModeQRTL : public Mode
{
public:

    Number mode_number() const override { return Number::QRTL; }
    const char *name() const override { return "QRTL"; }
    const char *name4() const override { return "QRTL"; }

    bool is_vtol_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    bool allows_arming() const override { return false; }

    bool does_auto_throttle() const override { return true; }

    bool update_target_altitude() override;

protected:

    bool _enter() override;
};

class ModeQAcro : public Mode
{
public:

    Number mode_number() const override { return Number::QACRO; }
    const char *name() const override { return "QACO"; }
    const char *name4() const override { return "QACRO"; }

    bool is_vtol_mode() const override { return true; }
    bool is_vtol_man_throttle() const override { return true; }
    virtual bool is_vtol_man_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
};

class ModeQAutotune : public Mode
{
public:

    Number mode_number() const override { return Number::QAUTOTUNE; }
    const char *name() const override { return "QAUTOTUNE"; }
    const char *name4() const override { return "QATN"; }

    bool is_vtol_mode() const override { return true; }
    virtual bool is_vtol_man_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
    void _exit() override;
};


class ModeTakeoff: public Mode
{
public:
    ModeTakeoff();

    Number mode_number() const override { return Number::TAKEOFF; }
    const char *name() const override { return "TAKEOFF"; }
    const char *name4() const override { return "TKOF"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_navigation() const override { return true; }

    bool does_auto_throttle() const override { return true; }

    AP_Int16 level_alt;

    // var_info for holding parameter information
    static const struct AP_Param::GroupInfo var_info[];

protected:
    AP_Int16 target_alt;
    AP_Int16 target_dist;
    AP_Int8 level_pitch;

    bool takeoff_started;
    Location start_loc;

    bool _enter() override;
};

#if HAL_SOARING_ENABLED

class ModeThermal: public Mode
{
public:

    Number mode_number() const override { return Number::THERMAL; }
    const char *name() const override { return "THERMAL"; }
    const char *name4() const override { return "THML"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // Update thermal tracking and exiting logic.
    void update_soaring();

    void navigate() override;

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_navigation() const override { return true; }

    // true if we are in an auto-throttle mode, which means
    // we need to run the speed/height controller
    bool does_auto_throttle() const override { return true; }

protected:

    bool exit_heading_aligned() const;
    void restore_mode(const char *reason, ModeReason modereason);

    bool _enter() override;
};

#endif


#if HAL_STALL_RECOVERY_ENABLED
class ModeStallRecovery : public Mode
{
public:
    Number mode_number() const override { return Number::STALLRECOVERY; }
    const char *name() const override { return "STALL"; }
    const char *name4() const override { return "STALL"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // false during STAGE 1, True during STAGE 2 if throttleMax2 < == -1
    bool does_auto_throttle() const override;

    void backup_mode_details();

    // var_info for holding parameter information
    static const struct AP_Param::GroupInfo var_info[];

    // public params for detection
    AP_Int32    detection_bitmask;
    AP_Int32    recovery_enable;

protected:
    bool _enter() override;
    void _exit() override;

    struct {
        // private-ish params for recovery
        AP_Int8     throttle1;
        AP_Int8     throttle2;
        AP_Int8     elevator;
        AP_Float    duration1_max;
        AP_Float    duration2_max;
        AP_Float    duration1_min;
        AP_Float    duration2_min;
        AP_Int32    algorithm1;
        AP_Int32    algorithm2;
        AP_Float    sink_rate;
        AP_Float    spin_rate;
    } param;

private:
    enum class STALL_RECOVERY_1 : uint32_t {
        AIRSPEED_MIN                = (1<<0),
        SPIN_RATE_PARAM             = (1<<1),
        SINK_RATE_PARAM             = (1<<2),
    };

    enum class STALL_RECOVERY_2 : uint32_t {
        AIRSPEED_CRUISE_100PCT      = (1<<0),
        AIRSPEED_CRUISE_95PCT       = (1<<1),
        AIRSPEED_CRUISE_90PCT       = (1<<2),
    };

    struct {
        Location wp_next;
        Location wp_prev;
    } backup;
    
    void set_servo_behavior();
    bool is_recovered_early() const;
    void resume_previous_mode();
};
#endif

