#pragma once

#include <Eigen/Dense>
#include "constants.h"

namespace Kalman {
    using Observation = Eigen::Matrix<double, SlamArraySize::OBSERVATION_DIM, 1>;
    using ObservationWithTag = Eigen::Matrix<double, SlamArraySize::OBSERVATION_DIM + 1, 1>;
    using ObservationCovariance = Eigen::Matrix<double, SlamArraySize::OBSERVATION_DIM, SlamArraySize::OBSERVATION_DIM>;
    using Input = Eigen::Matrix<double, SlamArraySize::INPUT_DIM, 1>;
    using InputCovariance = Eigen::Matrix<double, SlamArraySize::INPUT_DIM, SlamArraySize::INPUT_DIM>; // inputs to the model, e.g (wheel angular velocity, wheel orientation, wheel radius noise for deformation)
    using InputCovarianceTransfer = Eigen::Matrix<double, SlamArraySize::VEHICLE_STATE_DIM, SlamArraySize::INPUT_DIM>; // to transfer input covariance to vehicle state

    using VehicleState = Eigen::Matrix<double, SlamArraySize::VEHICLE_STATE_DIM, 1>;
    using VehicleStateCovariance = Eigen::Matrix<double, SlamArraySize::VEHICLE_STATE_DIM, SlamArraySize::VEHICLE_STATE_DIM>;
    using State = Eigen::Matrix<double, Eigen::Dynamic, 1>;
    using StateCovariance = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;

    using VehicleToObservationTransition = Eigen::Matrix<double, SlamArraySize::OBSERVATION_DIM, SlamArraySize::VEHICLE_STATE_DIM>;
    using LandmarkStateToObservationTransition = Eigen::Matrix<double, SlamArraySize::OBSERVATION_DIM, SlamArraySize::OBSERVATION_DIM>;

    using FullStateToInovationTransition = Eigen::Matrix<double, SlamArraySize::OBSERVATION_DIM, Eigen::Dynamic>;
    using KalmanGain = Eigen::Matrix<double,Eigen::Dynamic, SlamArraySize::OBSERVATION_DIM>;

    using Submatrix = Eigen::Block<Eigen::MatrixXd>;

    // extracts the 4x4 vehicle state covariance from
    // the large X matrice (that also includes landmarks)
    inline Submatrix XVehicleBlock(Eigen::MatrixXd& X) {
        return X.block(0, 0, SlamArraySize::VEHICLE_STATE_DIM, SlamArraySize::VEHICLE_STATE_DIM);
    }

    inline Submatrix XVehicleAndStateBlockTallAndSkinny(Eigen::MatrixXd& X) {
        return X.block(0, 0, X.rows(), SlamArraySize::VEHICLE_STATE_DIM);
    }

    // extracts the 4xN cross covariance matrix between vehicle and landmarks (N being number of landmarks)
    inline Submatrix VehicleToLandmarkCrossCovariance(Eigen::MatrixXd& X) {
        return X.block(0, SlamArraySize::VEHICLE_STATE_DIM,
                       SlamArraySize::VEHICLE_STATE_DIM,
                       X.rows() - SlamArraySize::VEHICLE_STATE_DIM);
    }

    // extracts the Nx4 cross covariance matrix between landmarks and vehicle (N being number of landmarks)
    inline Submatrix LandmarkToVehicleCrossCovariance(Eigen::MatrixXd& X) {
        return X.block(SlamArraySize::VEHICLE_STATE_DIM,0,
                       X.rows() - SlamArraySize::VEHICLE_STATE_DIM,
                       SlamArraySize::VEHICLE_STATE_DIM);
    }

    // extracts the 2x4 part of the H matrix relating to the vehicle states
    inline FullStateToInovationTransition VehicleStateToInnovationCovariance(FullStateToInovationTransition& H) {
        return H.block(0,0, SlamArraySize::OBSERVATION_DIM, SlamArraySize::VEHICLE_STATE_DIM);
    }

    // extracts the 2x2 part of the H matrix relating to the observed landmark
    inline FullStateToInovationTransition LandmarkStateToInnovationCovariance(FullStateToInovationTransition& H, const int index) {
        return H.block(0,index, SlamArraySize::OBSERVATION_DIM, SlamArraySize::OBSERVATION_DIM);
    }
}
