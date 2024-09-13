#pragma once

#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <unordered_set>
#include <Eigen/Dense>
#include <iomanip>  // For std::setw and std::setprecision

#include "types.h"
#include "functions.h"


class KalmanFilter;


class Logger{
public:
    explicit Logger();

    ~Logger();

    void log(int k, const std::string& message);
    void setStep(int step);

    void logKalmanFilterDataAfterPrediction(KalmanFilter* filter, const Kalman::Input& u);
    void logKalmanFilterDataAfterUpdate(KalmanFilter* filter);

    void saveAllData();

    template <typename MatrixType>
    void logMatrix(int k, const std::string& label, const MatrixType& matrix);

    template <typename VectorType>
    void logVector(int k, const std::string& label, const VectorType& vector);

    template <typename MatrixType>
    void logMatrixInMatlabVariableFormat(const std::string& label, const MatrixType& matrix, long int k);

    template <typename MatrixType>
    void logMatrixInMatlabVariableFormatSingleOperation(std::ofstream& file, const std::string& label, const MatrixType& matrix, long int k);

private:
    std::ofstream _logFile;
    int _currentStep;   // To keep track of the simulation step
    int _lastStep;      // To introduce a line break

    const std::string _directory;
    const std::string _log_file_name;

    std::unordered_set<std::string> _openedFiles;

    DataMatrix _x_vehicle_pred;
    DataMatrix _x_vehicle_est;
    DataMatrix _x_vehicle_pred_stdev;
    DataMatrix _x_vehicle_est_stdev;
    DataMatrix _u_noisy;
    DataMatrix _innovation;
    DataMatrix _innovation_cov;
    DataMatrix _z_obs;
    DataMatrix _z_pred;
    DataVector _chi2;
};
