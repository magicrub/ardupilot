#pragma once

#define EIGEN_NO_MALLOC
#define EIGEN_RUNTIME_NO_MALLOC
#define EIGEN_UNROLLING_LIMIT 0
#define EIGEN_NO_DEBUG
#define EIGEN_MALLOC_ALREADY_ALIGNED true

#ifdef _AP_CONFIG_H_
    #include "AP_BattMonitor_config.h"
#else
    #define BATTERY_EKF_ENABLED 1
#endif

#if BATTERY_EKF_ENABLED

#include <stdio.h>

#ifdef _AP_CONFIG_H_
    #if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    // Eigen gets angry about a missing declaration for std::vsnprintf() for HAL_BOARD_CHIBIOS builds
    namespace std {
    int vsnprintf(char *str, size_t size, const char *fmt, va_list ap);
    }
    #endif

    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    #include <AP_Math/Eigen/Dense>
    #pragma GCC diagnostic pop

#else
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    #include <Eigen/Dense>
    #pragma GCC diagnostic pop
#endif

#include "BatteryChemistryModel.h"

#include <stdint.h>

#define SQ(x) ((x)*(x))
#define SECONDS_PER_HOUR 3600

// STATES
#define STATE_IDX_SOC 0
#define STATE_IDX_SOH_INV 1
#define STATE_IDX_V1 2
#define STATE_IDX_V2 3
#define STATE_IDX_R0 4
#define STATE_IDX_R1 5
#define STATE_IDX_R2 6
#define N_STATES 7

class BatteryEKF {
public:
    // Typedefs
    typedef Eigen::Matrix<double,N_STATES,1> MatrixSx1;
    typedef Eigen::Matrix<double,N_STATES,N_STATES> MatrixSxS;

    typedef struct {
        float SOH_init;
        float R0_init;
        float R1_init;
        float R2_init;
        float RC1;
        float RC2;
        float I_sigma;
        float SOH_sigma;
        float R0_sigma;
        float R1_sigma;
        float R2_sigma;
        float I_step_sigma;
        float V_sigma;
        float Q;
        float R0_pnoise;
        float R1_pnoise;
        float R2_pnoise;
    } Params;

    BatteryEKF(Params& params, BatteryChemistryModel& model) :
    _params(params), _model(model)
    {}

    void set_params(Params params) {
        _params = params;
    }
    void set_chemistry_model(BatteryChemistryModel& model) {
        _model = model;
    }


    void initialize(float V, float I, float temp_C);
    void predict(float dt, float I);
    bool update(float V, float I, float temp_C, float& y, float& NIS);

    const MatrixSx1& get_state() {
        return x;
    }

    const MatrixSxS& get_covariance() {
        return P;
    }

    bool initialized() const {
        return _is_initialized;
    }

    float get_remaining_energy_J(float temp_C);
    float get_remaining_energy_Wh(float temp_C) {
        return get_remaining_energy_J(temp_C)/SECONDS_PER_HOUR;
    }

    float get_remaining_energy_J_sigma(float temp_C);
    float get_remaining_energy_Wh_sigma(float temp_C) {
        return get_remaining_energy_J_sigma(temp_C)/SECONDS_PER_HOUR;
    }

private:
    void addProcessNoise();

    bool _is_initialized;

    Params& _params;
    BatteryChemistryModel& _model;

    // EKF state
    MatrixSx1 x;
    MatrixSxS P;
};
#endif // BATTERY_EKF_ENABLED
