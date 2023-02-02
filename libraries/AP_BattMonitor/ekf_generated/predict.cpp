    { //////// Begin generated code: Prediction model      ////////
        double subx0 = exp(-dt/_params.RC1);
        double subx1 = I*(1 - subx0);
        double subx2 = exp(-dt/_params.RC2);
        double subx3 = 1 - subx2;
        double subx4 = I*subx3;
        double subx5 = ((_params.I_sigma)*(_params.I_sigma));
        double subx6 = 1/(((x(STATE_IDX_SOH))*(x(STATE_IDX_SOH))));
        double subx7 = I*dt*subx6/_params.Q;
        double subx8 = P(0,1) + P(1,1)*subx7;
        double subx9 = P(0,5) + P(1,5)*subx7;
        double subx10 = subx5*x(STATE_IDX_R1)*(1 - subx0);
        double subx11 = dt/(_params.Q*x(STATE_IDX_SOH));
        double subx12 = P(0,6) + P(1,6)*subx7;
        double subx13 = P(1,2)*subx0 + P(1,5)*subx1;
        double subx14 = P(1,3)*subx2 + P(1,6)*subx4;
        double subx15 = P(2,5)*subx0 + P(5,5)*subx1;
        double subx16 = P(2,6)*subx0 + P(5,6)*subx1;
        double subx17 = P(3,5)*subx2 + P(5,6)*subx4;
        double subx18 = P(3,6)*subx2 + P(6,6)*subx4;

        x_n(0,0) = -I*dt/(_params.Q*x(STATE_IDX_SOH)) + x(STATE_IDX_SOC);
        x_n(1,0) = x(STATE_IDX_SOH);
        x_n(2,0) = subx0*x(STATE_IDX_V1) + subx1*x(STATE_IDX_R1);
        x_n(3,0) = subx2*x(STATE_IDX_V2) + subx4*x(STATE_IDX_R2);
        x_n(4,0) = x(STATE_IDX_R0);
        x_n(5,0) = x(STATE_IDX_R1);
        x_n(6,0) = x(STATE_IDX_R2);

        P_n(0,0) = P(0,0) + P(0,1)*subx7 + _params.I_sigma*dt*subx6*(2*sqrt(P(0,0))*_params.Q*x(STATE_IDX_SOH) + _params.I_sigma*dt)/((_params.Q)*(_params.Q)) + subx7*subx8 + ((dt)*(dt))*subx5*subx6/((_params.Q)*(_params.Q));
        P_n(0,1) = subx8;
        P_n(0,2) = subx0*(P(0,2) + P(1,2)*subx7) + subx1*subx9 - subx10*subx11;
        P_n(0,3) = -subx11*subx3*subx5*x(STATE_IDX_R2) + subx12*subx4 + subx2*(P(0,3) + P(1,3)*subx7);
        P_n(0,4) = P(0,4) + P(1,4)*subx7;
        P_n(0,5) = subx9;
        P_n(0,6) = subx12;
        P_n(1,0) = subx8;
        P_n(1,1) = P(1,1);
        P_n(1,2) = subx13;
        P_n(1,3) = subx14;
        P_n(1,4) = P(1,4);
        P_n(1,5) = P(1,5);
        P_n(1,6) = P(1,6);
        P_n(2,0) = P(0,2)*subx0 + P(0,5)*subx1 - subx10*subx11 + subx13*subx7;
        P_n(2,1) = subx13;
        P_n(2,2) = subx0*(P(2,2)*subx0 + P(2,5)*subx1) + subx1*subx15 + subx5*((x(STATE_IDX_R1))*(x(STATE_IDX_R1)))*((1 - subx0)*(1 - subx0));
        P_n(2,3) = subx10*subx3*x(STATE_IDX_R2) + subx16*subx4 + subx2*(P(2,3)*subx0 + P(3,5)*subx1);
        P_n(2,4) = P(2,4)*subx0 + P(4,5)*subx1;
        P_n(2,5) = subx15;
        P_n(2,6) = subx16;
        P_n(3,0) = P(0,3)*subx2 + P(0,6)*subx4 - subx11*subx3*subx5*x(STATE_IDX_R2) + subx14*subx7;
        P_n(3,1) = subx14;
        P_n(3,2) = subx0*(P(2,3)*subx2 + P(2,6)*subx4) + subx1*subx17 + subx10*subx3*x(STATE_IDX_R2);
        P_n(3,3) = subx18*subx4 + subx2*(P(3,3)*subx2 + P(3,6)*subx4) + ((subx3)*(subx3))*subx5*((x(STATE_IDX_R2))*(x(STATE_IDX_R2)));
        P_n(3,4) = P(3,4)*subx2 + P(4,6)*subx4;
        P_n(3,5) = subx17;
        P_n(3,6) = subx18;
        P_n(4,0) = P(0,4) + P(1,4)*subx7;
        P_n(4,1) = P(1,4);
        P_n(4,2) = P(2,4)*subx0 + P(4,5)*subx1;
        P_n(4,3) = P(3,4)*subx2 + P(4,6)*subx4;
        P_n(4,4) = P(4,4);
        P_n(4,5) = P(4,5);
        P_n(4,6) = P(4,6);
        P_n(5,0) = subx9;
        P_n(5,1) = P(1,5);
        P_n(5,2) = subx15;
        P_n(5,3) = subx17;
        P_n(5,4) = P(4,5);
        P_n(5,5) = P(5,5);
        P_n(5,6) = P(5,6);
        P_n(6,0) = subx12;
        P_n(6,1) = P(1,6);
        P_n(6,2) = subx16;
        P_n(6,3) = subx18;
        P_n(6,4) = P(4,6);
        P_n(6,5) = P(5,6);
        P_n(6,6) = P(6,6);
    } //////// End generated code: Prediction model        ////////
