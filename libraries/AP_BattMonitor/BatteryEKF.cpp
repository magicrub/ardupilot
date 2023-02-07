#ifdef _AP_CONFIG_H_
    // this is stupid, but this must be included HERE and
    // not in BatteryEKF.h or else it gets included multiple
    // times and causes redefinition compile errors. This is
    // needed to define CONFIG_HAL_BOARD so we know to declare
    // std::vsnprintf()
    #include <AP_HAL/AP_HAL_Boards.h>
#endif

#include "BatteryEKF.h"

#if BATTERY_EKF_ENABLED
using namespace Eigen;

float BatteryEKF::get_remaining_energy_J(float temp_C) {
    return _params.Q*_model.OCV_from_SOC_integral(x(STATE_IDX_SOC), temp_C)/x(STATE_IDX_SOH_INV);
}

float BatteryEKF::get_remaining_energy_J_sigma(float temp_C) {
    return sqrt((P(0,0)*pow(x(STATE_IDX_SOH_INV), 2)*pow(_model.OCV_from_SOC(x(STATE_IDX_SOC), temp_C), 2) - 2*P(0,1)*x(STATE_IDX_SOH_INV)*_model.OCV_from_SOC(x(STATE_IDX_SOC), temp_C)*_model.OCV_from_SOC_integral(x(STATE_IDX_SOC), temp_C) + P(1,1)*pow(_model.OCV_from_SOC_integral(x(STATE_IDX_SOC), temp_C), 2))/pow(x(STATE_IDX_SOH_INV), 4))*fabs(_params.Q);
}


void BatteryEKF::initialize(float V, float I, float temp_C) {
    x.setZero();
    P.setZero();

    MatrixSx1& initial_state = x;
    MatrixSxS& initial_state_cov = P;

    #include "ekf_generated/init.cpp"

    _is_initialized = true;
}

void BatteryEKF::predict(float dt, float I) {
    MatrixSx1 x_n;
    MatrixSxS P_n;

    #include "ekf_generated/predict.cpp"

    P_n(STATE_IDX_R0,STATE_IDX_R0) += SQ(dt*_params.R0_pnoise);
    P_n(STATE_IDX_R1,STATE_IDX_R1) += SQ(dt*_params.R1_pnoise);
    P_n(STATE_IDX_R2,STATE_IDX_R2) += SQ(dt*_params.R2_pnoise);

    x = x_n;
    P = P_n;
}

bool BatteryEKF::update(float V, float I, float temp_C, float& y, float& NIS) {
    MatrixSx1 x_n;
    MatrixSxS P_n;

    #include "ekf_generated/update.cpp"

    if (NIS >= 3) {
        return false;
    }

    x = x_n;
    P = P_n;
    return true;
}
#endif //BATTERY_EKF_ENABLED
