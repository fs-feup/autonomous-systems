#pragma once

namespace VehicleModelConstants {
    constexpr double eps_k = 1e-6;
    constexpr double eps_y = 1e-6;

    constexpr double LONGVL = 10.0; 
    constexpr double LMUY = 1.0;
    constexpr double LVY = 1.0;
    constexpr double LMUV = 0.0;
    constexpr double LFZO = 1.0;

    constexpr double LKYC = 1;
    constexpr double LCY = 1.0;
    constexpr double LEY = 1.0;
    constexpr double LYKA = 1.0;
    constexpr double LVYKA = 1.0;
    constexpr double LKY = 1.0;

    constexpr double NOMPRES = 83000;
    constexpr double FNOM = 1110;

    constexpr double PPY1 = 0.43899;
    constexpr double PPY2 = 1.3335 ;
    constexpr double PPY3 = -0.15166;
    constexpr double PPY4 = 0.053855;
    constexpr double PPY5 = -0.81712;

    constexpr double PVY1 = 0.047901;
    constexpr double PVY2 = 0.014419;
    constexpr double PVY3 = -0.3046;
    constexpr double PVY4 = 1.4794;

    constexpr double PKY1 = -39.1199;
    constexpr double PKY2 = 1.6728;
    constexpr double PKY3 = 0.86703;
    constexpr double PKY4 = 2;
    constexpr double PKY5 = 29.7896;
    constexpr double PKY6 = -4.7914;
    constexpr double PKY7 = -2.0621;

    constexpr double PDY1 = 2.3352;
    constexpr double PDY2 = -0.37521;
    constexpr double PDY3 = 10;

    constexpr double PHY1 = 0.00013682;
    constexpr double PHY2 = -0.00046485;

    constexpr double PEY1 = 0.68111;
    constexpr double PEY2 = -0.35401;
    constexpr double PEY3 = -0.080852;
    constexpr double PEY4 = -5.562;
    constexpr double PEY5 = -64.1838;

    constexpr double RCY1 = 1.0;

    constexpr double RBY1 = 5.0;
    constexpr double RBY2 = 2;
    constexpr double RBY3 = 0.02;
    constexpr double RBY4 = 0;

    constexpr double REY1 = -0.1;
    constexpr double REY2 = 0.1;

    constexpr double RHY1 = 0;
    constexpr double RHY2 = 0;

    constexpr double RVY1 = 0;
    constexpr double RVY2 = 0;
    constexpr double RVY3 = 0;
    constexpr double RVY4 = 0;
    constexpr double RVY5 = 0;
    constexpr double RVY6 = 0;

    constexpr double PCY1 = 1.8528;

    constexpr double slip_ratio = 0;
    constexpr double y_input = 0;
    constexpr double p_input = 82737;
    constexpr double V_cx = 0; // not zero but it gets multiplied by 0;

    // Calculate the lateral force coefficient (bounded at -1 to 1) using the tire model parameters and slip angle
    // return std::sin(Clat * std::atan(Blat * alpha - Elat * (Blat * alpha - std::atan(Blat * alpha))));

    // Is this supposed to be 1?
    constexpr double zeta_0 = 1;
    constexpr double zeta_4 = 1.0; //ones(size(Fz));
    constexpr double zeta_3 = 1.0; //ones(size(Fz));
    constexpr double zeta_2 = 1.0; //ones(size(Fz));
}