#pragma once

#include <Eigen/Dense>
#include "constants.h"
#include <stdlib.h>     /* used for EXIT_SUCCESS */
#include <iostream>  // Required for std::cerr
#include <iomanip> // For std::setw and std::setprecision
#include <filesystem> // For C++17 or later

using namespace std;

using DataMatrix = std::vector<std::vector<double>>;
using DataVector = std::vector<double>;

namespace Kalman {
    using Observation = Eigen::Matrix<double, SLAM_ARRAY_SIZE::OBSERVATION_DIM, 1>;
    using ObservationWithTag = Eigen::Matrix<double, SLAM_ARRAY_SIZE::OBSERVATION_DIM + 1, 1>;  // +1 for the beacon Id tag
    using ObservationCovariance = Eigen::Matrix<double, SLAM_ARRAY_SIZE::OBSERVATION_DIM, SLAM_ARRAY_SIZE::OBSERVATION_DIM>;
    using Input = Eigen::Matrix<double, SLAM_ARRAY_SIZE::INPUT_DIM, 1>;
    using InputCovariance = Eigen::Matrix<double, SLAM_ARRAY_SIZE::INPUT_DIM + 1, SLAM_ARRAY_SIZE::INPUT_DIM + 1>; // inputs to the model, e.g (wheel angular velocity, wheel orientation, wheel radius noise for deformation)
    using InputCovarianceTransfer = Eigen::Matrix<double, SLAM_ARRAY_SIZE::VEHICLE_STATE_DIM, SLAM_ARRAY_SIZE::INPUT_DIM + 1>; // to transfer input covariance to vehicle state (+1 to feed wheel radius noise through)

    using VehicleState = Eigen::Matrix<double, SLAM_ARRAY_SIZE::VEHICLE_STATE_DIM, 1>;
    using VehicleStateCovariance = Eigen::Matrix<double, SLAM_ARRAY_SIZE::VEHICLE_STATE_DIM, SLAM_ARRAY_SIZE::VEHICLE_STATE_DIM>;
    using State = Eigen::Matrix<double, Eigen::Dynamic, 1>;
    using StateCovariance = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;

    using VehicleToObservationTransition = Eigen::Matrix<double, SLAM_ARRAY_SIZE::OBSERVATION_DIM, SLAM_ARRAY_SIZE::VEHICLE_STATE_DIM>;
    using LandmarkStateToObservationTransition = Eigen::Matrix<double, SLAM_ARRAY_SIZE::OBSERVATION_DIM, SLAM_ARRAY_SIZE::OBSERVATION_DIM>;

    using FullStateToInovationTransition = Eigen::Matrix<double, SLAM_ARRAY_SIZE::OBSERVATION_DIM, Eigen::Dynamic>;
    using KalmanGain = Eigen::Matrix<double,Eigen::Dynamic, SLAM_ARRAY_SIZE::OBSERVATION_DIM>;

    using Submatrix = Eigen::Block<Eigen::MatrixXd>;


    // Helper function to check the state of matrices in the Kalman Filter
    inline void checkMatrixState(
        const Eigen::MatrixXd& H,
        const Eigen::MatrixXd& _Hv,
        const Eigen::MatrixXd& _Hp,
        const Eigen::MatrixXd& _X,
        const Eigen::MatrixXd& _sigma_z,
        const Eigen::VectorXd& _x
    ) {

        // Check for NaN or Inf values in matrices
        if (H.hasNaN() || _Hv.hasNaN() || _Hp.hasNaN() || _X.hasNaN() || _sigma_z.hasNaN() || _x.hasNaN()) {
            std::cerr << "Error: One or more matrices contain NaN values!" << std::endl;
        }

        if (!H.allFinite() || !_Hv.allFinite() || !_Hp.allFinite() || !_X.allFinite() || !_sigma_z.allFinite() || !_x.allFinite()) {
            std::cerr << "Error: One or more matrices contain Inf values!" << std::endl;
        }

        // Check if matrix H has appropriate dimensions
        if (H.rows() != SLAM_ARRAY_SIZE::OBSERVATION_DIM || H.cols() != _x.rows()) {
            std::cerr << "Error: Matrix H has incorrect dimensions!" << std::endl;
        }

        // Check if _Hv and _Hp have appropriate dimensions
        if (_Hv.rows() != SLAM_ARRAY_SIZE::OBSERVATION_DIM || _Hv.cols() != SLAM_ARRAY_SIZE::VEHICLE_STATE_DIM) {
            std::cerr << "Error: Matrix _Hv has incorrect dimensions!" << std::endl;
        }

        if (_Hp.rows() != SLAM_ARRAY_SIZE::OBSERVATION_DIM || _Hp.cols() != SLAM_ARRAY_SIZE::OBSERVATION_DIM) {
            std::cerr << "Error: Matrix _Hp has incorrect dimensions!" << std::endl;
        }

        // Check if _X and _sigma_z have square dimensions
        if (_X.rows() != _X.cols()) {
            std::cerr << "Error: Matrix _X is not square!" << std::endl;
        }

        if (_sigma_z.rows() != _sigma_z.cols()) {
            std::cerr << "Error: Matrix _sigma_z is not square!" << std::endl;
        }
    }

    // extracts the 4x4 vehicle state covariance from
    // the large X matrice (that also includes landmarks)
    inline Submatrix XVehicleBlock(Eigen::MatrixXd& X)
    {
        return X.block(0, 0, SLAM_ARRAY_SIZE::VEHICLE_STATE_DIM, SLAM_ARRAY_SIZE::VEHICLE_STATE_DIM);
    }

    inline Submatrix XVehicleAndStateBlockTallAndSkinny(Eigen::MatrixXd& X)
    {
        return X.block(0, 0, X.rows(), SLAM_ARRAY_SIZE::VEHICLE_STATE_DIM);
    }

    // extracts the 4xN cross covariance matrix between vehicle and landmarks (N being number of landmarks)
    // will throw an error is
    inline Submatrix VehicleToLandmarkCrossCovariance(Eigen::MatrixXd& X)
    {
        if (X.rows() <= SLAM_ARRAY_SIZE::VEHICLE_STATE_DIM )
        {
            std::cerr << "Cannot extract submatrix when there are no landmarks!  " << __func__ << " (in " << __FILE__ << ")" << std::endl;
            std::exit(EXIT_FAILURE);
        }
            return X.block(0, SLAM_ARRAY_SIZE::VEHICLE_STATE_DIM,
                       SLAM_ARRAY_SIZE::VEHICLE_STATE_DIM,
                       X.rows() - SLAM_ARRAY_SIZE::VEHICLE_STATE_DIM);
    }

    // extracts the Nx4 cross covariance matrix between landmarks and vehicle (N being number of landmarks)
    inline Submatrix LandmarkToVehicleCrossCovariance(Eigen::MatrixXd& X)
    {
        if (X.rows() <= SLAM_ARRAY_SIZE::VEHICLE_STATE_DIM )
        {
            std::cerr << "Cannot extract submatrix when there are no landmarks!  " << __func__ << " (in " << __FILE__ << ")" << std::endl;
            std::exit(EXIT_FAILURE);
        }
        return X.block(SLAM_ARRAY_SIZE::VEHICLE_STATE_DIM,0,
                       X.rows() - SLAM_ARRAY_SIZE::VEHICLE_STATE_DIM,
                       SLAM_ARRAY_SIZE::VEHICLE_STATE_DIM);
    }

    // extracts the 2x4 part of the H matrix relating to the vehicle states
    inline Eigen::Block<Eigen::MatrixXd> VehicleStateToInnovationCovariance(Eigen::MatrixXd& H)
    {
        return H.block(0, 0, SLAM_ARRAY_SIZE::OBSERVATION_DIM, SLAM_ARRAY_SIZE::VEHICLE_STATE_DIM);
    }

    // extracts the 2x2 part of the H matrix relating to the observed landmark
    inline Eigen::Block<Eigen::MatrixXd> LandmarkStateToInnovationCovariance(Eigen::MatrixXd& H, const int index)
    {
        return H.block(0, index, SLAM_ARRAY_SIZE::OBSERVATION_DIM, SLAM_ARRAY_SIZE::OBSERVATION_DIM);
    }
}
