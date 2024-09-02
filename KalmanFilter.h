#pragma once

#include <Eigen/Dense>

#include "constants.h"
#include "functions.h"
#include <math.h>
#include <iostream>

#include <random>
#include <cmath>

#include "types.h"

using namespace std;




class CKalmanFilter
{
public:
    CKalmanFilter();
    void CorruptControls(Eigen::VectorXd utrue, Eigen::VectorXd& u);
    void PredictState(Kalman::Input& u);
    void UpdateEKF(Kalman::ObservationWithTag& z);
    bool MatchObservation(Eigen::VectorXd& obs, Eigen::MatrixXd& Beacons);
    void AddState(Eigen::VectorXd& z);
    bool isMapped(Eigen::VectorXd& obsRB);

    // public member function that allow to set and modify the noise parameters of the Kalman Filter
    void ModifyAlongTrackNoiseMultiplicative( const double& sigma_q) { m_var_q = sigma_q*sigma_q; };
    void ModifyAlongTrackNoiseAdditive( const double& sigma_w) { m_var_w = sigma_w*sigma_w; };
    void ModifyCrossTrackNoiseMultiplicative( const double& sigma_s) { m_var_s = sigma_s*sigma_s; };
    void ModifyCrossTrackNoiseAdditive( const double& sigma_g ) { m_var_g = sigma_g*sigma_g; };
    void ModifyWheelRadiusNoiseAdditive( const double& sigma_r ) { m_var_r = sigma_r*sigma_r; };
    void ModifyObservationNoiseRangeAdditive( const double& sigma_range ) { m_var_range = sigma_range*sigma_range; };
    void ModifyObservationNoiseBearingAdditive( const double& sigma_bearing) { m_var_bearing = sigma_bearing*sigma_bearing; };

    // To get member data
    double GetSigmaRange(){ return m_wStDevRange; };
    double GetSigmaTheta(){ return m_wStDevTheta; };

protected:
    // Noise model parameters for the Process Noise v and its Covariance Matrix Q
    double m_vStDevVehicleLocXY;	// Standard deviation of the noise v on the XY coordinates in m
    double m_vStDevVehicleLocTheta;	// Standard deviation of the noise v on the orientation in rad
    // radius is missing !!!!!
    double m_vStDevBeaconLoc;		// Standard deviation of the noise v on the beacon location in m
    bool   m_QEnabled;				// Is Process noise v added to the predicted state?

    // Noise model parameters for the Observation Noise w an its Covariance Matrix R
    double m_wStDevRange;				// Standard deviation of the noise w on the first coordinate of z
    double m_wStDevTheta;				// Standard deviation of the noise w on the second coordinate of z
    bool   m_REnabled;				// Is Observation noise w added ?

    // Note for Q and R: these matrix are meant to accomodate unmodeled uncertainies
    // which are entering the system. You'll typically add a random gaussian noise
    // to the state vector and its associated covariance matrix to achieve this.
    // They are different from the uncertain inputs covariance matrix Sigma_u and the
    // Observation covariance matrix Sigma_z !!!
    // If you decide not to use these feature, set the boolean values
    // m_QEnabled and m_Renabled to false

    // Noise model parameters for the control and observation uncertainties
    double m_var_q;				// Variance of the multicative along track noise (slip) in (%)^2
    double m_var_w;				// Variance of the additive along track noise (slip) in (rad/s)^2
    double m_var_s;				// Variance of the multicative cross track noise (skip) in (%)^2
    double m_var_g;				// Variance of the additive cross track noise (skip) in (rad)^2
    double m_var_r;				// Variance of the additive wheel radius noise (deformation) in m^2
    double m_var_range;			// Variance of the additive range noise in (m)^2
    double m_var_bearing;		// Variance of the additive bearing noise in (rad)^2

    vector<int> m_mappedLandmarkList;

private:

    // Initialise with only vehicle state. Features will be added online
    Kalman::State  _x = Kalman::State::Zero(SlamArraySize::VEHICLE_STATE_DIM);
    Kalman::StateCovariance  _X = Kalman::StateCovariance ::Zero(SlamArraySize::VEHICLE_STATE_DIM,SlamArraySize::VEHICLE_STATE_DIM);

    // Input error covariance (e.g: 3x3 if 3 inputs) - used in PredictState()
    Kalman::InputCovariance _sigma_u = Kalman::InputCovariance::Zero();

    // Input error covariance transfer matrix (e.g: 4x3 if 4 states and 3 inputs) - used in PredictState()
    Kalman::InputCovarianceTransfer _G = Kalman::InputCovarianceTransfer::Zero();

    // State transition matrix (4x4 if 4 states) - used in PredictState()
    Kalman::VehicleStateCovariance _F = Kalman::VehicleStateCovariance::Zero();

    // Transfer vehicle state covariance into observation covariance (e.g: 2 x 4 if 2D observation & 4 vehicle states)
    Kalman::VehicleToObservationTransition _Hv = Kalman::VehicleToObservationTransition::Zero();

    // Transfer landmark state covariance into observation covariance (e.g: 2 x 2 if 2D observation)
    Kalman::LandmarkStateToObservationTransition _Hp = Kalman::LandmarkStateToObservationTransition::Zero();

    // 2x1 vector of zeros (predicted measurement)
    Kalman::Observation _zpred =  Kalman::Observation::Zero();

    // 2x1 vector of zeros (measurement residual)
    Kalman::Observation _v =  Kalman::Observation::Zero();

    // 2x2 matrix of zeros (measurement covariance)
    Kalman::ObservationCovariance _sigma_z = Kalman::ObservationCovariance::Zero();

    // 2x2 matrix of zeros (innovation covariance)
    Kalman::ObservationCovariance _S = Kalman::ObservationCovariance::Zero();

    // Initialize Tx as 2x4  - used in AddState()
    Kalman::ObservationCovariance _Tx = Kalman::ObservationCovariance::Zero();

    // Initialize Tz as 2x2  - used in AddState()
    Kalman::ObservationCovariance _Tz = Kalman::ObservationCovariance::Zero();

    // Initialize R (Noise covariance matrix) as 2x2 - used in AddState()
    Kalman::ObservationCovariance _R = Kalman::ObservationCovariance::Zero();

    // Initialize obsWRF (Observation matrix) as 2x1 - used in AddState()
    Kalman::Observation _obsWRF = Kalman::Observation::Zero();

    double _dt = 0.0;

};
