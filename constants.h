#pragma once

#define WORLD_SIZE 500	// 0-100 meters in each direction
#define LINC 0.1			// 0.1m length for spline interpolation
#define DT 0.1				// Sample interval for controller
#define TEND 1000000*DT	// Total maximum run time for simulator
#define VVEL 2				// Vehicle velocity (assumed constant)

#define KP 1				// vehicle position error gain
#define KO 1				// vehicle orientation error gain

#define WHEEL_BASE 1		// vehicle wheel base (m)
#define WHEEL_RADIUS 0.3   // nomial wheel radius (m)
#define R_RATE 12.566		// rotation rate (rads/s)

#define XOFFSET 0
#define YOFFSET 0

#define RAD2DEG 360/(2*pi)

#define PLOT_INNO 0

#define PATHWINDOW 20

const int XTRUE_DIM = 4;
const int UTRUE_DIM = 3;

#define U_CONTROLS_DIM 4
#define X_VEHICLE_DIM 4

const int REFPATH_DIM = 2;
const int BEACON_DIM = 2;
const int U_DIM = 3;
const int WRF_OBS_DIM = 4;
const int RB_OBS_DIM = 4;

#define R_MAX_RANGE 100			// Maximum range of the radar


const double VAR_XX = 0.024;
const double VAR_YY = 0.024;
const double VAR_TT = 0.0003;
const double VAR_RR = 0.0001;

const double SIGMA_Q = 0.25;		// Multiplicative Slip Error std deviation in percent (%)
const double SIGMA_W = 0.1;			// Additive Slip Error std deviation in rad/s
const double SIGMA_S = 0.01;		// Multiplicative Skid error std deviation in percent (%)
const double SIGMA_G = 0.00873;		// Additive Skid error std deviation in rad

const double PI = 3.14159265358979;

const double DISTANCE_GAP_FOR_CLOSING_LOOP = 0.2;	// In CVehicleModel::ComputeControl()
