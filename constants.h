#pragma once

// File directory for outputs (used in constructor of Logger)
constexpr const char* FILE_OUTPUT_DIR = "../cpp/outputs/";

// File for general outputs (as opposed to single variables in matrix / vector format
constexpr const char* FILE_NAME_FOR_GENERAL_OUTPUTS = "slam_simulation_output.txt";

// File paths for inputs to the simulation: target path (X,Y) and beacon location (X, Y)
constexpr const char* FILE_REFPATH = "../cpp/inputs/RefPath_small_5Loops.txt";
constexpr const char* FILE_BEACONS = "../cpp/inputs/Beacons_small_5Loops.txt";

namespace SLAM_PHYSICAL_CONST{
    constexpr double WHEEL_BASE     = 1.0;		// vehicle wheel base (m)
    constexpr double WHEEL_RADIUS   = 0.3;      // nomial wheel radius (m)
    constexpr double R_RATE         = 12.566;   // rotation angular velocity  of radar (rads/s)
    constexpr double R_MAX_RANGE    = 20;      // Maximum range of the radar

    constexpr int XOFFSET           = 0;        // see kinematic model of vehicle
    constexpr int YOFFSET           = 0;        // see kinematic model of vehicle
    constexpr int WORLD_SIZE        = 500;      // 0-100 meters in each direction
}

namespace SLAM_ARRAY_SIZE {
    constexpr int VEHICLE_STATE_DIM     = 4;   // The 4 states modelling the vehicle (X, Y, Orientation, Wheel radius)
    constexpr int INPUT_DIM             = 2;   // The 3 inputs: (Wheel angular velocity, Wheel orientation)
    constexpr int OBSERVATION_DIM       = 2;   // (X, Y) of each beacons in the world reference frame, or (range, bearing) in vehicle RF
    constexpr int OBSERVATION_SIM_DIM   = 3;   // (X,Y) or (range, bearing) + the beacon ID that was observed (this is for simulated observations)
    constexpr int VEHICLE_PATH_DIM      = 2;   // (X, Y) for a single point of the vehicle path loaded from file
}

namespace SLAM_NOISE {
    constexpr double VAR_XX = 0.024;        // variance of vehicle x location (m^2). Only used at initialisation and not as process noise!
    constexpr double VAR_YY = 0.024;        // variance of vehicle y location (m^2). Only used at initialisation and not as process noise!
    constexpr double VAR_TT = 0.0003;       // variance of vehicle orientation (rad^2). Only used at initialisation and not as process noise!
    constexpr double VAR_RR = 0.0001;       // variance of wheel radius noise (m^2). Only used at initialisation and not as process noise!

    constexpr double SIGMA_Q = 0.25;		// Multiplicative Slip Error std deviation in percent (%) - process noise
    constexpr double SIGMA_W = 0.1;			// Additive Slip Error std deviation in rad/s - process noise
    constexpr double SIGMA_S = 0.01;		// Multiplicative Skid error std deviation in percent (%) - process noise
    constexpr double SIGMA_G = 0.00872665;  // Additive Skid error std deviation in rad - process noise
    constexpr double SIGMA_R = 0.005;		// wheel radius noise standard deviation (m) - process noise

    constexpr double SIGMA_RANGE = 0.3;		// Observation range standard deviation (m) - observation noise
    constexpr double SIGMA_BEARING = 0.0349;// Observation bearing standard deviation (rad) - observation noise
}

namespace SLAM_CONST{
    constexpr double LINC       = 0.1;              // 0.1m length for spline interpolation
    constexpr double DT         = 0.1;				// Sample interval for controller
    constexpr double TEND       = 1000000*DT;       // Total maximum run time for simulator
    constexpr double VVEL       = 2;				// Vehicle velocity (assumed constant)
    constexpr double PI         = 3.14159265358979;
    constexpr double KP         = 1;                // vehicle position error gain
    constexpr double KO         = 1;                // vehicle orientation error gain
    constexpr double RAD2DEG    = 360/(2*PI);
    constexpr int INIT_STEPS    = 300;              // number of steps at the start for collecting observations
    constexpr int PLOT_INNO     = 0;
    constexpr int NUM_LOOPS     = 1;                // how many loops to run as defined in FILE_REFPATH
    constexpr double PATHWINDOW = 20;
    constexpr double DISTANCE_GAP_FOR_CLOSING_LOOP = 0.2;	// In VehicleModel::ComputeControl()
    constexpr double CHI_SQUARED                   = 7.5;   // Innovation gate
}
