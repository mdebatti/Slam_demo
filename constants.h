#pragma once

namespace SlamPhysicalConstants{
    constexpr double WHEEL_BASE     = 1.0;		// vehicle wheel base (m)
    constexpr double WHEEL_RADIUS   = 0.3;      // nomial wheel radius (m)
    constexpr double R_RATE         = 12.566;   // rotation rate (rads/s)
    constexpr double R_MAX_RANGE    = 100;      // Maximum range of the radar

    constexpr int XOFFSET           = 0;
    constexpr int YOFFSET           = 0;
    constexpr int WORLD_SIZE        = 500;      // 0-100 meters in each direction
}

namespace SlamArraySize {
    constexpr int VEHICLE_STATE_DIM     = 4;   // The 4 states modelling the vehicle (X, Y, Orientation, Wheel radius)
    constexpr int INPUT_DIM             = 3;   // The 3 inputs: (Wheel angular velocity, Wheel orientation, Wheel radius noise) - wheel radius noise is to model deformation
    constexpr int OBSERVATION_DIM       = 2;   // (X, Y) of each beacons in the world reference frame

    constexpr int VEHICLE_PATH_DIM      = 4;   // (X, Y, Orientation, Time) for a single point of the vehicle path loaded from file
    constexpr int VEHICLE_CONTROLS_DIM  = 3;   // (Wheel angular velocity, steering angle, Time) for a single point of the vehicle controls saved to file
}

namespace SlamNoise {
    constexpr double VAR_XX = 0.024;
    constexpr double VAR_YY = 0.024;
    constexpr double VAR_TT = 0.0003;
    constexpr double VAR_RR = 0.0001;

    constexpr double SIGMA_Q = 0.25;		// Multiplicative Slip Error std deviation in percent (%)
    constexpr double SIGMA_W = 0.1;			// Additive Slip Error std deviation in rad/s
    constexpr double SIGMA_S = 0.01;		// Multiplicative Skid error std deviation in percent (%)
    constexpr double SIGMA_G = 0.00873;		// Additive Skid error std deviation in rad
}

namespace SlamConstants{
    constexpr double LINC       = 0.1;              // 0.1m length for spline interpolation
    constexpr double DT         = 0.1;				// Sample interval for controller
    constexpr double TEND       = 1000000*DT;       // Total maximum run time for simulator
    constexpr double VVEL       = 2;				// Vehicle velocity (assumed constant)
    constexpr double PI         = 3.14159265358979;
    constexpr double KP         = 1;                // vehicle position error gain
    constexpr double KO         = 1;                // vehicle orientation error gain
    constexpr double RAD2DEG    = 360/(2*PI);
    constexpr int PLOT_INNO     = 0;
    constexpr double PATHWINDOW = 20;
    constexpr double DISTANCE_GAP_FOR_CLOSING_LOOP = 0.2;	// In CVehicleModel::ComputeControl()
}
