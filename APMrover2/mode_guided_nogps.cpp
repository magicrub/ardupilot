#include "mode.h"
#include "Rover.h"

bool ModeGuidedNoGPS::_enter()
{
    // initialise waypoint speed
    set_desired_speed_to_default();

    // set desired location to reasonable stopping point
    calc_stopping_location(_destination);
    set_desired_location(_destination);

    return true;
}

// void ModeGuidedNoGPS::update()
// {
//     // if (_guided_mode != ModeGuided::Guided_TurnRateAndSpeed) {
//     //     // force this state and halt the vehicle if we're in any other state
//     //     set_desired_turn_rate_and_speed(0,0);
//     //     gcs().send_text(MAV_SEVERITY_DEBUG, "Switching to Guided_TurnRateAndSpeed");
//     //     return;
//     // }

// update

//     // same as guided.update switch _guided_mode == Guided_TurnRateAndSpeed
//     update_TurnRateAndSpeed();
// }

