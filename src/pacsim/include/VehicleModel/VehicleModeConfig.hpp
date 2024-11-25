#pragma once

namespace VehicleModelConstants {
    constexpr double eps_k = 1e-6;
    constexpr double eps_y = 1e-6;

    constexpr double LONGVL = 10; 
    constexpr double LMUY = 1.0;
    constexpr double LVY = 0;
    constexpr double LMUV = 0;
    constexpr double LFZO = 1.0;

    constexpr double LKYC = 1;
    constexpr double LCY = 1.0;
    constexpr double LEY = 1.0;
    constexpr double LYKA = 1.0;
    constexpr double LVYKA = 1.0;
    constexpr double LKY = 1.0;

    constexpr double NOMPRES = 100000;
    constexpr double FNOM = 1500;

    constexpr double PPY1 = 0.1;
    constexpr double PPY2 = 0.1;
    constexpr double PPY3 = 0;
    constexpr double PPY4 = 0;
    constexpr double PPY5 = 0;

    constexpr double PVY1 = 0;
    constexpr double PVY2 = 0;
    constexpr double PVY3 = 0;
    constexpr double PVY4 = 0;

    constexpr double PKY1 = -20;
    constexpr double PKY2 = 2;
    constexpr double PKY3 = 0;
    constexpr double PKY4 = 2;
    constexpr double PKY5 = 0;
    constexpr double PKY6 = 0;
    constexpr double PKY7 = 0;

    constexpr double PDY1 = 0.8;
    constexpr double PDY2 = -0.05;
    constexpr double PDY3 = 0;

    constexpr double PHY1 = 0;
    constexpr double PHY2 = 0;

    constexpr double PEY1 = -0.8;
    constexpr double PEY2 = -0.6;
    constexpr double PEY3 = 0.1;
    constexpr double PEY4 = 0;
    constexpr double PEY5 = 0;

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

    constexpr double PCY1 = 2.0;

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