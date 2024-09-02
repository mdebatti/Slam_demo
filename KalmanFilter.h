#pragma once

#include <Eigen/Dense>

#include "constants.h"
#include "functions.h"
#include <math.h>
#include <iostream>
using namespace std;

#include <random>
#include <cmath>




class CKalmanFilter
{
public:
    CKalmanFilter();
    void CorruptControls(Eigen::VectorXd utrue, Eigen::VectorXd& u);
    void PredictState(Eigen::VectorXd& x, Eigen::MatrixXd& X, Eigen::VectorXd& u);
    void UpdateEKF(Eigen::VectorXd& x, Eigen::MatrixXd& X, Eigen::VectorXd& z);
    bool MatchObservation(Eigen::VectorXd& obs, Eigen::MatrixXd& Beacons);
    void AddState(Eigen::VectorXd& x, Eigen::MatrixXd& X, Eigen::VectorXd& z);
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

};
