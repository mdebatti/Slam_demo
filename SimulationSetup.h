#pragma once

#include <random> // Required for random_device, mt19937, normal_distribution
#include "types.h"
#include "FileIO.h"
#include "functions.h"

class SimulationSetup
{
public:
    SimulationSetup(const char* refpathFile, const char* beaconFile,  int numLoops = 1);
    Kalman::VehicleState getInitialVehicleState();

    int get_num_steps() const {return _num_steps;};
    int get_num_beacons() const {return _num_beacons;};

    Kalman::Input getControlInputsNoisy(int k) const;
    Kalman::ObservationWithTag getNoisyObservationWithTag(int k) const;
    double getWheelRadiusNoise(int k) const;

private:
    std::pair<double, double> calcVehiclePositionAndOrientationError(int k) const;
    DataMatrix storeFileToVectorOfVectors (const char* path_to_file, int expected_num_cols) const;
    void calcAndSaveControlAndTrueVehicleState(int k, const std::pair<double, double> positionAndOrientationErrors);
    void generateControlsAndTrueVehicleState();
    void generateProcessAndMeasurementNoise();
    void simulateObservationsAlongPath();
    void resetObservations(int rr);
    void resizeObservationVectors(int index);

    int _num_path_points;   // number of points in refpathFile
    int _num_steps;         // number of steps  (_num_path_points X SLAM_CONST::NUM_LOOPS)
    int _num_beacons;       // number of beacons in beaconFile

    DataMatrix _vehicle_state_true;                       // (x, y, orientation, wheel radius), generated from _ref_path by computeControlsAndTrueVehicleState()
    DataMatrix _vehicle_controls_clean;                   // wheel angular velocity & wheel steering angle (no noise)
    DataMatrix _vehicle_controls_noisy;                   // wheel angular velocity & wheel steering angle (with noise)
    DataMatrix _beacons_observationsRB_clean;
    DataMatrix _beacons_observationsRB_noise;
    DataMatrix _beacons_observationsRB_noisy;
    DataMatrix _beacons_observationsWRF_clean;
    DataMatrix _beacons_observationsWRF_noisy;
    DataVector _beacons_is_observed;

    DataMatrix _ref_path;                                 // reference (x,y) vehicle path, from which _vehicle_state_true is generated
    DataMatrix _beacons_location;
    DataVector _vehicle_wheel_radius_state_additive_noise;// to be added to previous wheel radius estimate at the kalman filter prediction step (double check why)
    DataVector _time;                                     // simulation time

    int _numLoops;
};


