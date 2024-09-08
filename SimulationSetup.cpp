#include "SimulationSetup.h"

// Loads and allocate all the member variable of the class VehicleModel thanks
// to the file. It is always assumed that one variable is one column of the file
SimulationSetup::SimulationSetup(const char* refpathFile, const char* beaconFile, int numLoops) : _numLoops(numLoops)
{   
    std::cout << "Starting to setup simulation environment for EKF Slam demo..." << endl;

    // Load the path reference from file (the "target" path of the vehicle)
    _ref_path = storeFileToVectorOfVectors (refpathFile, SLAM_ARRAY_SIZE::VEHICLE_PATH_DIM);
    _beacons_location = storeFileToVectorOfVectors (beaconFile, SLAM_ARRAY_SIZE::OBSERVATION_DIM);
    _num_path_points = _ref_path.size();
    _num_steps = SLAM_CONST::NUM_LOOPS*_num_path_points;
    _num_beacons = _beacons_location.size();

    // Create and fill the time vector used in the simulation (for radar observation)
    _time.resize(_num_steps);
    for (int i = 0; i < _num_steps; ++i) {
        _time[i] = i * SLAM_CONST::DT;
    }

    // generate the actual path of the vehicle as well as
    // controls (wheel angular velocity and wheel steering)
    // this will fill "_vehicle_state_true" & "_vehicle_controls_clean"
    generateControlsAndTrueVehicleState();

    // Generate process noise for simulation
    // fill "_vehicle_wheel_radius_state_additive_noise" and "_vehicle_controls_noisy"
    generateProcessAndMeasurementNoise();

    // simulate all observations, along with measurement noise
    simulateObservationsAlongPath();

    std::cout << "Finished setting up simulation environment for EKF Slam demo!" << endl << endl;
}

Kalman::Input SimulationSetup::getControlInputsNoisy(int k) const
{
    Kalman::Input u = Kalman::Input::Zero();

    for(int ii = 0; ii < SLAM_ARRAY_SIZE::INPUT_DIM; ++ii)
    {
        u(ii) = _vehicle_controls_noisy[k][ii];
    }
    return u;
}

Kalman::ObservationWithTag SimulationSetup::getNoisyObservationWithTag(int k) const
{
    Kalman::ObservationWithTag y = Kalman::ObservationWithTag::Zero();

    for(int ii = 0; ii < SLAM_ARRAY_SIZE::OBSERVATION_SIM_DIM; ++ii)
    {
        y(ii) = _beacons_observationsRB_noisy[k][ii];
    }
    return y;
}

double SimulationSetup::getWheelRadiusNoise(int k) const
{
    return _vehicle_wheel_radius_state_additive_noise[k];
}

DataMatrix SimulationSetup::storeFileToVectorOfVectors (const char* path_to_file, int expected_num_cols) const
{
    // opens the file and check it
    FileIO file_to_import(path_to_file);

    // return a DataMatrix object of the expected size
    return file_to_import.getDataMatrix(expected_num_cols);
}

Kalman::VehicleState SimulationSetup::getInitialVehicleState()
{
    Kalman::VehicleState initial_state = Kalman::VehicleState::Zero();

    initial_state(0) = _vehicle_state_true[0][0];
    initial_state(1) = _vehicle_state_true[0][1];
    initial_state(2) = _vehicle_state_true[0][2];
    initial_state(3) = _vehicle_state_true[0][3];

    return initial_state;
};


void SimulationSetup::generateProcessAndMeasurementNoise()
{
    // Set up the random number generator
    std::random_device rd;  // Seed generator (non-deterministic random device)
    std::mt19937 gen(rd()); // Mersenne Twister RNG initialized with the seed
    std::normal_distribution<> normal_dist(0.0, 1.0); // Gaussian with mean 0 and stddev 1

    // Process noise for the control inputs
    _vehicle_controls_noisy.resize(_num_steps);

    // Measurement noise for the observations
    _beacons_observationsRB_noise.resize(_num_steps);

    // Process noise for wheel radius
    _vehicle_wheel_radius_state_additive_noise.resize(_num_steps);

    for (int rr = 0; rr < _num_steps; ++rr)
    {
        double u0 = _vehicle_controls_clean[rr][0];
        double u1 = _vehicle_controls_clean[rr][1];

        int n_cols = _vehicle_controls_clean[rr].size();
        _vehicle_controls_noisy[rr].resize(n_cols);

        // Generate Gaussian noise for wheel angular velocity
        double noise_q = normal_dist(gen); // Gaussian noise for wheel angular velocity
        double noise_w = normal_dist(gen); // Gaussian noise for control inputs
        _vehicle_controls_noisy[rr][0] = u0 + SLAM_NOISE::SIGMA_Q * noise_q * u0 + SLAM_NOISE::SIGMA_W * noise_w;

        // Generate Gaussian noise for steer angle
        double noise_s = normal_dist(gen); // Gaussian noise for steer angle
        double noise_g = normal_dist(gen); // Gaussian noise for control inputs
        _vehicle_controls_noisy[rr][1] = u1 + SLAM_NOISE::SIGMA_S * noise_s * u1 + SLAM_NOISE::SIGMA_G * noise_g;

        // Process noise for wheel radius
        _vehicle_wheel_radius_state_additive_noise[rr] = SLAM_CONST::DT*SLAM_NOISE::SIGMA_R * normal_dist(gen);

        // Measurement noise for the observations
        _beacons_observationsRB_noise.resize(_num_steps);
        _beacons_observationsRB_noise[rr].resize(SLAM_ARRAY_SIZE::OBSERVATION_DIM);
        _beacons_observationsRB_noise[rr][0] = SLAM_NOISE::SIGMA_RANGE * normal_dist(gen);
        _beacons_observationsRB_noise[rr][1] = SLAM_NOISE::SIGMA_BEARING * normal_dist(gen);

        //std::cout << "U noisy should be:\t" << _vehicle_controls_noisy[rr][0] << " \t" << _vehicle_controls_noisy[rr][1] << std::endl;
    }
}

void SimulationSetup::generateControlsAndTrueVehicleState()
{
    // resize the first dimension of these double vectors
    _vehicle_state_true.resize(_num_steps);
    _vehicle_controls_clean.resize(_num_steps);
    for(int rr = 0; rr < _num_steps; ++rr)
    {
        _vehicle_state_true[rr].resize(SLAM_ARRAY_SIZE::VEHICLE_STATE_DIM);
        _vehicle_controls_clean[rr].resize(SLAM_ARRAY_SIZE::INPUT_DIM);
    }

    // Initialisation with the initial position of the vehicle (X, Y, Theta, R)
    _vehicle_state_true[0][0] = _ref_path[0][0];
    _vehicle_state_true[0][1] = _ref_path[0][1];
    _vehicle_state_true[0][2] = atan2( _ref_path[1][1] - _ref_path[0][1], _ref_path[1][0] - _ref_path[0][0] );
    _vehicle_state_true[0][3] = SLAM_PHYSICAL_CONST::WHEEL_RADIUS;

    // Initialisation of control vector [ Wheel angular velocity, steering angle]
    _vehicle_controls_clean[0][0] = SLAM_CONST::VVEL;
    _vehicle_controls_clean[0][1] = 0.0;

    // Control Loop - The true path and true input are stocked in xtrue and utrue
    // should start with k = 1 as there is an access to k-1 in calcAndSaveControlAndTrueVehicleState
    for(int k = 1; k < _num_steps; ++k)
    {
        // At time k, get the error between actual position and reference path
        // Use it to compute the new control vector for time k+1 along
        // with the new position for time k+1.
        calcAndSaveControlAndTrueVehicleState(k, calcVehiclePositionAndOrientationError(k-1));
    }

    // Convert the vehicle speed in rad/s
    for(int rr = 0; rr < _num_steps; ++rr )
    {
        _vehicle_controls_clean[rr][0] = _vehicle_controls_clean[rr][0]/SLAM_PHYSICAL_CONST::WHEEL_RADIUS;
    }

    saveDataToFile(_vehicle_controls_clean, generateNewFilename("vehicle_controls_clean", FILE_REFPATH));
    saveDataToFile(_vehicle_state_true, generateNewFilename("vehicle_state_true", FILE_REFPATH));
}

std::pair<double, double> SimulationSetup::calcVehiclePositionAndOrientationError(const int k) const
{
    double vehicle_x = _vehicle_state_true[k][0];
    double vehicle_y = _vehicle_state_true[k][1];
    double vehicle_orientation = _vehicle_state_true[k][2];

    int closestIndex = -1;
    double minDistance = std::numeric_limits<double>::max();

    // Iterate through the path points to find the closest one
    // (not looking for efficiency here...)
    for (int ii = 0; ii < _num_path_points; ++ii)
    {
        double dx = _ref_path[ii][0] - vehicle_x;
        double dy = _ref_path[ii][1] - vehicle_y;
        double distance = std::sqrt(dx * dx + dy * dy);

        if (distance < minDistance)
        {
            minDistance = distance;
            closestIndex = ii;
        }
    }

    // Assuming _num_path_points is the total number of points in the path
    int nextIndex = (closestIndex + 1) % _num_path_points;

    // Vector from the closest point to the next point on the path
    double deltaPathX = _ref_path[nextIndex][0] - _ref_path[closestIndex][0];
    double deltaPathY = _ref_path[nextIndex][1] - _ref_path[closestIndex][1];

    // Vector from the closest point on the path to the vehicle's current position
    double deltaVehicleX = vehicle_x - _ref_path[closestIndex][0];
    double deltaVehicleY = vehicle_y - _ref_path[closestIndex][1];

    // Position error (cross-track error)
    double crossTrackError = deltaPathY * deltaVehicleX - deltaPathX * deltaVehicleY;

    // Estimated heading (angle) of the path segment
    double estimatedHeading = atan2(deltaPathY, deltaPathX);

    // Orientation error (difference between estimated heading and vehicle's actual heading)
    double orientationError = normalizeAngle(estimatedHeading - vehicle_orientation);

    return std::make_pair(crossTrackError, orientationError);
}

void SimulationSetup::calcAndSaveControlAndTrueVehicleState(const int k,
                                                            const std::pair<double, double> positionAndOrientationErrors)
{
    if (k < 1) {
        std::cerr << "Intput parameter k in  " << __func__ << " (in " << __FILE__ << ") was less than 1 (disallowed as there is access to [k-1])" << std::endl;
        std::exit(EXIT_FAILURE);
    }

    double perr = positionAndOrientationErrors.first;
    double oerr = positionAndOrientationErrors.second;

    // The control vector utrue = [v, steer]' (for now, convert to [wheel angular velocity, steer] later)
    _vehicle_controls_clean[k][0] = _vehicle_controls_clean[k-1][0];
    _vehicle_controls_clean[k][1] = normalizeAngle( SLAM_CONST::KP*perr +SLAM_CONST::KO*oerr);

    double x_prev = _vehicle_state_true[k-1][0];
    double y_prev = _vehicle_state_true[k-1][1];
    double or_prev = _vehicle_state_true[k-1][2];
    double u_wheel_lin_vel = _vehicle_controls_clean[k][0];
    double u_wheel_steer = _vehicle_controls_clean[k][1];

    // The true state vector of the vehicle _vehicle_state_true = [x,y,theta,R]
    _vehicle_state_true[k][0] = x_prev + SLAM_CONST::DT*u_wheel_lin_vel*cos( or_prev + u_wheel_steer);
    _vehicle_state_true[k][1] = y_prev + SLAM_CONST::DT*u_wheel_lin_vel*sin( or_prev + u_wheel_steer);
    _vehicle_state_true[k][2] = or_prev +SLAM_CONST::DT*u_wheel_lin_vel*sin(u_wheel_steer) / SLAM_PHYSICAL_CONST::WHEEL_BASE;
    _vehicle_state_true[k][3] = _vehicle_state_true[k-1][3];
}


// Simulate radar observations from the vehicle between a start location xtrue_start and
// an end position xtrue_end based on the true beacon location and the true vehicle state xtrue.
// When multiple beacons are observed, only the nearest one is recorded.
//
// The location vectors xtrue_start and xtrue_end each consist of a vector of [x,y,phi,t]
// describing the location of the vehicle at a given time.
// The beacon map is an array of locations
//
// ObsWRF are the observation in world reference frame, ObsVRF are in vehicle reference frame and
// the function returns either true or false if it observed something or not


void SimulationSetup::resetObservations(int stepIndex)
{
    _beacons_observationsRB_clean[stepIndex][0] = 0.0;
    _beacons_observationsRB_clean[stepIndex][1] = 0.0;
    _beacons_observationsRB_clean[stepIndex][2] = -1;  // Set index to -1 (no observation)

    _beacons_observationsRB_noisy[stepIndex][0] = 0.0;
    _beacons_observationsRB_noisy[stepIndex][1] = 0.0;
    _beacons_observationsRB_noisy[stepIndex][2] = -1;  // Set index to -1 (no observation)

    _beacons_observationsWRF_clean[stepIndex][0] = 0.0;
    _beacons_observationsWRF_clean[stepIndex][1] = 0.0;
    _beacons_observationsWRF_clean[stepIndex][2] = -1;  // Set index to -1 (no observation)

    _beacons_observationsWRF_noisy[stepIndex][0] = 0.0;
    _beacons_observationsWRF_noisy[stepIndex][1] = 0.0;
    _beacons_observationsWRF_noisy[stepIndex][2] = -1;  // Set index to -1 (no observation)
}

void SimulationSetup::resizeObservationVectors(int stepIndex)
{
    const int dimension = SLAM_ARRAY_SIZE::OBSERVATION_SIM_DIM;
    _beacons_observationsRB_clean[stepIndex].resize(dimension, 0.0);
    _beacons_observationsRB_noisy[stepIndex].resize(dimension, 0.0);
    _beacons_observationsWRF_clean[stepIndex].resize(dimension, 0.0);
    _beacons_observationsWRF_noisy[stepIndex].resize(dimension, 0.0);
}

void SimulationSetup::simulateObservationsAlongPath()
{
    _beacons_observationsRB_clean.resize(_num_steps);
    _beacons_observationsRB_noisy.resize(_num_steps);
    _beacons_observationsWRF_clean.resize(_num_steps);
    _beacons_observationsWRF_noisy.resize(_num_steps);
    _beacons_is_observed.resize(_num_steps);

    for(int stepIndex = 0; stepIndex < _num_steps - 1; ++stepIndex)
    {
        // Allocate space for range, bearing, index
        resizeObservationVectors(stepIndex);

        // Cos and Sin of the vehicle's orientation at start (1) and end (2) location
        double cosPhi1 = cos(_vehicle_state_true[stepIndex][2]);
        double sinPhi1 = sin(_vehicle_state_true[stepIndex][2]);

        double cosPhi2 = cos(_vehicle_state_true[stepIndex + 1][2]);
        double sinPhi2 = sin(_vehicle_state_true[stepIndex + 1][2]);

        // Find location of the radar at start (1) and end (2) locations
        double radarX1 = _vehicle_state_true[stepIndex][0] + (SLAM_PHYSICAL_CONST::XOFFSET * cosPhi1 - SLAM_PHYSICAL_CONST::YOFFSET * sinPhi1);
        double radarY1 = _vehicle_state_true[stepIndex][1] + (SLAM_PHYSICAL_CONST::XOFFSET * sinPhi1 + SLAM_PHYSICAL_CONST::YOFFSET * cosPhi1);

        double radarX2 = _vehicle_state_true[stepIndex + 1][0] + (SLAM_PHYSICAL_CONST::XOFFSET * cosPhi2 - SLAM_PHYSICAL_CONST::YOFFSET * sinPhi2);
        double radarY2 = _vehicle_state_true[stepIndex + 1][1] + (SLAM_PHYSICAL_CONST::XOFFSET * sinPhi2 + SLAM_PHYSICAL_CONST::YOFFSET * cosPhi2);

        // Normalized radar aim angles
        double radarPhi1 = normalizeAngle(_time[stepIndex] * SLAM_PHYSICAL_CONST::R_RATE);
        double radarPhi2 = normalizeAngle(_time[stepIndex + 1] * SLAM_PHYSICAL_CONST::R_RATE);

        // Radar aiming line segments
        double radarDx1 = SLAM_PHYSICAL_CONST::R_MAX_RANGE * cos(radarPhi1 + _vehicle_state_true[stepIndex][2]);
        double radarDy1 = SLAM_PHYSICAL_CONST::R_MAX_RANGE * sin(radarPhi1 + _vehicle_state_true[stepIndex][2]);

        double radarDx2 = SLAM_PHYSICAL_CONST::R_MAX_RANGE * cos(radarPhi2 + _vehicle_state_true[stepIndex + 1][2]);
        double radarDy2 = SLAM_PHYSICAL_CONST::R_MAX_RANGE * sin(radarPhi2 + _vehicle_state_true[stepIndex + 1][2]);

        double closestBeaconRange = SLAM_PHYSICAL_CONST::R_MAX_RANGE;
        int closestBeaconIndex = -1;

        for(int beaconIndex = 0; beaconIndex < _num_beacons; ++beaconIndex)
        {
            double dx = _beacons_location[beaconIndex][0] - radarX1;
            double dy = _beacons_location[beaconIndex][1] - radarY1;
            double range = sqrt(dx * dx + dy * dy);

            double dotProduct1 = ((dy * radarDx1) - (dx * radarDy1));
            double dotProduct2 = ((_beacons_location[beaconIndex][1] - radarY2) * radarDx2) - ((_beacons_location[beaconIndex][0] - radarX2) * radarDy2);

            if((range < SLAM_PHYSICAL_CONST::R_MAX_RANGE) && (dotProduct1 > 0) && (dotProduct2 < 0) && (range < closestBeaconRange)) {
                closestBeaconRange = range;
                closestBeaconIndex = beaconIndex;
            }
        }

//        if((closestBeaconRange > 50.0) && (closestBeaconIndex > 0))
//        {
//            std::cout << "We have a problem!" << std::endl;
//        }

        if(closestBeaconIndex >= 0)
        {
            double vehicleOrientation = normalizeAngle(_vehicle_state_true[stepIndex][2]);
            _beacons_observationsRB_clean[stepIndex][0] = closestBeaconRange;
            _beacons_observationsRB_clean[stepIndex][1] = normalizeAngle(atan2(_beacons_location[closestBeaconIndex][1] - radarY1, _beacons_location[closestBeaconIndex][0] - radarX1) - vehicleOrientation);
            _beacons_observationsRB_clean[stepIndex][2] = closestBeaconIndex;

            // Add noise to observation
            _beacons_observationsRB_noisy[stepIndex][0] = _beacons_observationsRB_clean[stepIndex][0]; //+ _beacons_observationsRB_noise[stepIndex][0];
            _beacons_observationsRB_noisy[stepIndex][1] = _beacons_observationsRB_clean[stepIndex][1]; // + _beacons_observationsRB_noise[stepIndex][1];
            _beacons_observationsRB_noisy[stepIndex][2] = closestBeaconIndex;

            // Observation in world reference frame
            _beacons_observationsWRF_clean[stepIndex][0] = radarX1 + _beacons_observationsRB_clean[stepIndex][0] * cos(_beacons_observationsRB_clean[stepIndex][1] + _vehicle_state_true[stepIndex][2]);
            _beacons_observationsWRF_clean[stepIndex][1] = radarY1 + _beacons_observationsRB_clean[stepIndex][0] * sin(_beacons_observationsRB_clean[stepIndex][1] + _vehicle_state_true[stepIndex][2]);
            _beacons_observationsWRF_clean[stepIndex][2] = closestBeaconIndex;

            _beacons_observationsWRF_noisy[stepIndex][0] = radarX1 + _beacons_observationsRB_noisy[stepIndex][0] * cos(_beacons_observationsRB_noisy[stepIndex][1] + _vehicle_state_true[stepIndex][2]);
            _beacons_observationsWRF_noisy[stepIndex][1] = radarY1 + _beacons_observationsRB_noisy[stepIndex][0] * sin(_beacons_observationsRB_noisy[stepIndex][1] + _vehicle_state_true[stepIndex][2]);
            _beacons_observationsWRF_noisy[stepIndex][2] = closestBeaconIndex;
        } else
        {
            resetObservations(stepIndex);
        }
        /*
        std::cout << "observation (range, bearing, Index):\t"
                  << _beacons_observationsRB_noisy[stepIndex][0] << "\t"
                  << _beacons_observationsRB_noisy[stepIndex][1] << "\t"
                  << _beacons_observationsRB_noisy[stepIndex][2] << "\n";

        std::cout << "observation (X, Y, Index):\t"
                  << _beacons_observationsWRF_noisy[stepIndex][0] << "\t"
                  << _beacons_observationsWRF_noisy[stepIndex][1] << "\t"
                  << _beacons_observationsWRF_noisy[stepIndex][2] << "\n";
        */
    }

    // Handle the last step separately
    resizeObservationVectors(_num_steps - 1);
    resetObservations(_num_steps - 1);

    saveDataToFile(_beacons_observationsWRF_noisy, generateNewFilename("beacons_observationsWRF_noisy", FILE_REFPATH));
    saveDataToFile(_beacons_observationsRB_noisy, generateNewFilename("beacons_observationsRB_noisy", FILE_REFPATH));

}

template <typename T>
void saveDataToFile(const T& data, const std::string& filename)
{
    // Define the relative path to the outputs directory
     std::string outputDir = "../cpp/outputs/";

     // Create the directory if it doesn't exist
     std::filesystem::create_directories(outputDir);

     // Append the filename to the output directory path
     std::string fullPath = outputDir + filename;

     std::ofstream file(fullPath);
     if (!file.is_open())
     {
         std::cerr << "Failed to open file: " << fullPath << std::endl;
         return;
     }

    file.precision(20);

    // Handle 2D vector (DataMatrix)
    if constexpr (std::is_same_v<T, DataMatrix>)
    {
        for(int rr = 0; rr < data.size(); ++rr )
        {
            int n_cols = data[rr].size();
            for(int cc = 0; cc < n_cols; ++cc)
            {
                file << data[rr][cc];
                if (cc < n_cols - 1)  // Only add a space if it's not the last element
                {
                    file << " ";
                }
            }
            file << '\n';  // Move to the next line after all columns are written
        }
    }

    // Handle 1D vector (DataVector)
    if constexpr (std::is_same_v<T, DataVector>)
    {
        for(int rr = 0; rr < data.size(); ++rr )
        {
            file << data[rr] << '\n';
        }
    }

    file.close();
}

// Explicit template instantiation
template void saveDataToFile<DataMatrix>(const DataMatrix& data, const std::string& filename);
template void saveDataToFile<DataVector>(const DataVector& data, const std::string& filename);
