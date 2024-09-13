#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <random>
#include <cmath>
#include <math.h>

#include "constants.h"
#include "types.h"
#include "functions.h"
#include "Logger.h"
#include "KalmanFilterStrategy.h"
#include <optional>

class Logger;

class KalmanFilter : public KalmanFilterStrategy
{
public:
    KalmanFilter(Logger& logger,
                 const Kalman::VehicleState& x_init = Kalman::VehicleState::Zero(),
                 const Kalman::VehicleState& diag_X_init = Kalman::VehicleState(SLAM_NOISE::VAR_XX,
                                                                                SLAM_NOISE::VAR_YY,
                                                                                SLAM_NOISE::VAR_TT,
                                                                                SLAM_NOISE::VAR_RR),
                 double var_q = SLAM_NOISE::SIGMA_Q*SLAM_NOISE::SIGMA_Q,
                 double var_w = SLAM_NOISE::SIGMA_W*SLAM_NOISE::SIGMA_W,
                 double var_s = SLAM_NOISE::SIGMA_S*SLAM_NOISE::SIGMA_S,
                 double var_g = SLAM_NOISE::SIGMA_G*SLAM_NOISE::SIGMA_G,
                 double var_r = SLAM_NOISE::SIGMA_R*SLAM_NOISE::SIGMA_R,
                 double var_range = SLAM_NOISE::SIGMA_RANGE*SLAM_NOISE::SIGMA_RANGE,
                 double var_bearing = SLAM_NOISE::SIGMA_BEARING*SLAM_NOISE::SIGMA_BEARING);

    void predict (const Kalman::Input& u, double wheel_radius_noise) override;
    void update() override;
    bool addState() override;
    bool isMapped(const Kalman::ObservationWithTag& obsRB) override;

    DataVector getVehicleStates() const;
    DataVector getVehicleStateStdev() const;
    DataVector getInnovation() const;
    DataVector getMeasurement() const;
    DataVector getPredictedMeasurement() const;
    DataVector getInnovationCov() const;
    double get_chi2() const;

    // public member function that allow to set and modify the noise parameters of the Kalman Filter
    void setAlongTrackNoiseMultiplicativeStdev( const double& sigma_q) { _var_q = sigma_q*sigma_q; };
    void setAlongTrackNoiseAdditiveStdev( const double& sigma_w) { _var_w = sigma_w*sigma_w; };
    void setCrossTrackNoiseMultiplicativeStdev( const double& sigma_s) { _var_s = sigma_s*sigma_s; };
    void setCrossTrackNoiseAdditiveStdev( const double& sigma_g ) { _var_g = sigma_g*sigma_g; };
    void setWheelRadiusNoiseAdditiveStdev( const double& sigma_r ) { _var_r = sigma_r*sigma_r; };
    void setObservationNoiseRangeAdditiveStdev( const double& sigma_range ) { _var_range = sigma_range*sigma_range; };
    void setObservationNoiseBearingAdditiveStdev( const double& sigma_bearing) { _var_bearing = sigma_bearing*sigma_bearing; };

    // Logging to a file
    void logStateAndCovariance(int k, const std::string& message) const;
    void logInnovationAndCovariance(int k, const std::string& message) const;
    void logStateToInnovationTransferMatrixH(int k, const std::string& message) const;

    void logControlInput(int k, const Kalman::Input& u) const;
    void logObservation(int k, const Kalman::ObservationWithTag& z) const;
    void log(int k, const std::string& message) const;

    // Displaying to terminal
    void displayStateCovarianceMatrix(const std::optional<std::string>& message = std::nullopt, std::optional<int> k = -1) const;
    void displayHMatrix(const Kalman::FullStateToInovationTransition& H) const;
    void displayTransferToInnovationMatrix(const Kalman::ObservationCovariance& H, const std::string& label) const;
    void displayVehicleStateToInnovationTransferMatrix() const;
    void displaylandmarkStateToInnovationTransferMatrix() const;

    // Setter function for manipulating the EKF state for testing from main
    void set_x(const Kalman::State& x){_x = x;}
    void set_X(const Kalman::StateCovariance& X){_X = X;}
    void set_dt(double dt){_dt = dt;}
    void reset_K(){_K.setZero();};
    void reset_S(){_S.setZero();};
    void reset_Hv(){_Hv.setZero();};
    void reset_sigma_z(){_sigma_z.setZero();};
    void reset_Hp(){_Hp.setZero();};
    void reset_Tx(){_Tx.setZero();};
    void reset_Tz(){_Tz.setZero();};
    void reset_mu(){_mu.setZero();};
    void reset_sigma_new_landmark(){_sigma_new_landmark.setZero();};
    void set_z(const Kalman::ObservationWithTag& z){
        _H.resize(SLAM_ARRAY_SIZE::OBSERVATION_DIM, _x.rows());
        _z = z;
    }

    // Getter function to access the internal matrix of the KalmanFilter class for logging
    const Kalman::State& get_x() const { return _x; };
    const Kalman::StateCovariance& get_X() const { return _X; }
    const Kalman::KalmanGain& get_K() const { return _K; }
    const Kalman::ObservationCovariance& get_S() const { return _S; }
    const Kalman::ObservationWithTag& get_z() const { return _z; }
    const Kalman::Observation& get_obsWRF() const { return _obsWRF; }
    const Kalman::VehicleToObservationTransition& get_Tx() const { return _Tx; }
    const Kalman::ObservationCovariance& get_Tz() const { return _Tz; }
    const Kalman::StateCovariance& get_mu() const { return _mu; }
    const Kalman::ObservationCovariance& get_sigma_new_landmark() const { return _sigma_new_landmark; }
    const Kalman::FullStateToInovationTransition& get_H() const { return _H; }
    const Kalman::InputCovarianceTransfer& get_G() const { return _G; }
    const Kalman::VehicleStateCovariance& get_F() const { return _F; }
    const Kalman::InputCovariance& get_sigma_u() const { return _sigma_u; }
    const Kalman::VehicleToObservationTransition& get_Hv() const { return _Hv; }
    const Kalman::LandmarkStateToObservationTransition& get_Hp() const { return _Hp; }
    const Kalman::LandmarkStateToObservationTransition& get_sigma_z() const { return _sigma_z; }

private:
    // Matrix operations in predict() step
    void predictMatrixOperationsDo();

    // Matrix operations in update() step
    void updateMatrixOperationsDo();

    // test functions
    void testMatrixOperationsInPredict();
    void testMatrixOperationsInUpdate(const Kalman::State& x_init);

    // Function to convert an Eigen::VectorXd to std::vector<double>
    DataVector eigenToStdVector(const Eigen::VectorXd& eigenVec) const;

    // Noise model parameters for the control and observation uncertainties
    double _var_q;				// Variance of the multicative along track noise (slip) in (%)^2
    double _var_w;				// Variance of the additive along track noise (slip) in (rad/s)^2
    double _var_s;				// Variance of the multicative cross track noise (skip) in (%)^2
    double _var_g;				// Variance of the additive cross track noise (skip) in (rad)^2
    double _var_r;				// Variance of the additive wheel radius noise (deformation) in m^2
    double _var_range;			// Variance of the additive range noise in (m)^2
    double _var_bearing;		// Variance of the additive bearing noise in (rad)^2

    // Initialise with only vehicle state. Features will be added online
    Kalman::State  _x = Kalman::State::Zero(SLAM_ARRAY_SIZE::VEHICLE_STATE_DIM);
    Kalman::StateCovariance  _X = Kalman::StateCovariance ::Zero(SLAM_ARRAY_SIZE::VEHICLE_STATE_DIM,SLAM_ARRAY_SIZE::VEHICLE_STATE_DIM);

    // Input error covariance (e.g: 3x3 if 3 inputs) - used in PredictState()
    Kalman::InputCovariance _sigma_u = Kalman::InputCovariance::Zero();

    // Input error covariance transfer matrix (e.g: 4x3 if 4 states and 3 inputs) - used in PredictState()
    // note that there is 3 columns as we have noise from the inputs (wheel angular velocity and wheel steer)
    // and 1 column for the 'only' process noise affecting the vehicle state without a matrix transform
    // (G (3,2) = 1 and G(3,0) = G(3,1) = G(0,2) = G(1,2) = G(2,2) = 0 )
    Kalman::InputCovarianceTransfer _G = Kalman::InputCovarianceTransfer::Zero();

    // State transition matrix (4x4 if 4 states) - used in PredictState()
    Kalman::VehicleStateCovariance _F = Kalman::VehicleStateCovariance::Zero();

    // Jacobian of the prediction model with respect to the state (e.g: 2 x 4 if 2D observation & 4 vehicle states)
    Kalman::VehicleToObservationTransition _Hv = Kalman::VehicleToObservationTransition::Zero();

    // Jacobian of the prdiction model with respect to the landmark location (e.g: 2 x 2 if 2D observation)
    Kalman::LandmarkStateToObservationTransition _Hp = Kalman::LandmarkStateToObservationTransition::Zero();

    // Transfer state covariance into innovation covariance (e.g: 2 x N if 2D observation and N states)
    Kalman::FullStateToInovationTransition _H = Kalman::FullStateToInovationTransition::Zero(SLAM_ARRAY_SIZE::OBSERVATION_DIM,SLAM_ARRAY_SIZE::VEHICLE_STATE_DIM);

    // Observation with matching tag (e.g: 2+1 x 1 if 2D observation)
    Kalman::ObservationWithTag _z = Kalman::ObservationWithTag::Zero();

    // 2x1 vector of zeros (predicted measurement)
    Kalman::Observation _zpred =  Kalman::Observation::Zero();

    // 2x1 vector of zeros (measurement residual)
    Kalman::Observation _v =  Kalman::Observation::Zero();

    // Kalman gain is a dynamic matrix of the number of states x number of observations
    Kalman::KalmanGain _K;

    // 2x2 matrix of zeros (measurement covariance)
    Kalman::ObservationCovariance _sigma_z = Kalman::ObservationCovariance::Zero();

    // 2x2 matrix of zeros (innovation covariance)
    Kalman::ObservationCovariance _S = Kalman::ObservationCovariance::Zero();

    // Initialize Tx as 2x4  - used in AddState()
    Kalman::VehicleToObservationTransition _Tx = Kalman::VehicleToObservationTransition::Zero();

    // Initialize Tz as 2x2  - used in AddState()
    Kalman::ObservationCovariance _Tz = Kalman::ObservationCovariance::Zero();

    // mu for the existing state covariance when adding a landmark (dynamic matrix) - used in AddState
    Kalman::StateCovariance _mu = Kalman::StateCovariance ::Zero(SLAM_ARRAY_SIZE::VEHICLE_STATE_DIM,SLAM_ARRAY_SIZE::VEHICLE_STATE_DIM);

    // Covariance of new landmark - used in AddState
    Kalman::ObservationCovariance _sigma_new_landmark = Eigen::Matrix2d::Zero();

    // Initialize R (Noise covariance matrix) as 2x2 - used in AddState()
    Kalman::ObservationCovariance _R = Kalman::ObservationCovariance::Zero();

    // Initialize obsWRF (Observation matrix) as 2x1 - used in AddState()
    Kalman::Observation _obsWRF = Kalman::Observation::Zero();

    // Time step of simulation
    double _dt = 0.0;

    // For keeping track of which beacons have already been observed
    vector<int> _mappedLandmarkList;

    // Chi2 used as innovation gate
    double _chi2;

    // Subscripts for accessing observed landmark state and Covariance.
    int _px = 0;

    // For logging to a text file
    Logger& _logger;
};
