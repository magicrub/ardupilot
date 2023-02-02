    { //////// Begin generated code: Initial state covariance ////////
        float subx0 = I*_params.R0_init + I*_params.R1_init + I*_params.R2_init + V;
        float subx1 = // Not supported in C:
// _model.SOC_from_OCV_diff
_model.SOC_from_OCV_diff(subx0, temp_C);
        float subx2 = ((subx1)*(subx1));
        float subx3 = ((I)*(I));
        float subx4 = ((_params.R1_sigma)*(_params.R1_sigma));
        float subx5 = ((_params.R2_sigma)*(_params.R2_sigma));
        float subx6 = ((_params.I_step_sigma)*(_params.I_step_sigma));
        float subx7 = _params.R1_init + _params.R2_init;
        float subx8 = ((_params.I_sigma)*(_params.I_sigma));
        float subx9 = _params.R1_init*subx1*subx6*subx7 + _params.R1_init*subx1*subx8*(_params.R0_init + subx7) + subx1*subx3*subx4;
        float subx10 = _params.R2_init*subx1*subx6*subx7 + _params.R2_init*subx1*subx8*(_params.R0_init + subx7) + subx1*subx3*subx5;

        initial_state(0) = // Not supported in C:
// _model.SOC_from_OCV
_model.SOC_from_OCV(subx0, temp_C);
        initial_state(1) = _params.SOH_init;
        initial_state(2) = I*_params.R1_init;
        initial_state(3) = I*_params.R2_init;
        initial_state(4) = _params.R0_init;
        initial_state(5) = _params.R1_init;
        initial_state(6) = _params.R2_init;

        initial_state_cov(0,0) = ((_params.R0_sigma)*(_params.R0_sigma))*subx2*subx3 + ((_params.V_sigma)*(_params.V_sigma))*subx2 + subx2*subx3*subx4 + subx2*subx3*subx5 + subx2*subx6*((subx7)*(subx7)) + subx2*subx8*((_params.R0_init + subx7)*(_params.R0_init + subx7));
        initial_state_cov(0,2) = subx9;
        initial_state_cov(0,3) = subx10;
        initial_state_cov(0,4) = I*((_params.R0_sigma)*(_params.R0_sigma))*subx1;
        initial_state_cov(0,5) = I*subx1*subx4;
        initial_state_cov(0,6) = I*subx1*subx5;
        initial_state_cov(1,1) = ((_params.SOH_sigma)*(_params.SOH_sigma));
        initial_state_cov(2,0) = subx9;
        initial_state_cov(2,2) = ((_params.R1_init)*(_params.R1_init))*subx6 + ((_params.R1_init)*(_params.R1_init))*subx8 + subx3*subx4;
        initial_state_cov(2,3) = _params.R1_init*_params.R2_init*subx6 + _params.R1_init*_params.R2_init*subx8;
        initial_state_cov(2,5) = I*subx4;
        initial_state_cov(3,0) = subx10;
        initial_state_cov(3,2) = _params.R1_init*_params.R2_init*subx6 + _params.R1_init*_params.R2_init*subx8;
        initial_state_cov(3,3) = ((_params.R2_init)*(_params.R2_init))*subx6 + ((_params.R2_init)*(_params.R2_init))*subx8 + subx3*subx5;
        initial_state_cov(3,6) = I*subx5;
        initial_state_cov(4,0) = I*((_params.R0_sigma)*(_params.R0_sigma))*subx1;
        initial_state_cov(4,4) = ((_params.R0_sigma)*(_params.R0_sigma));
        initial_state_cov(5,0) = I*subx1*subx4;
        initial_state_cov(5,2) = I*subx4;
        initial_state_cov(5,5) = subx4;
        initial_state_cov(6,0) = I*subx1*subx5;
        initial_state_cov(6,3) = I*subx5;
        initial_state_cov(6,6) = subx5;
    } //////// End generated code: Initial state covariance   ////////
