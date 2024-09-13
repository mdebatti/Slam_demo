#include "Logger.h"
#include "KalmanFilter.h"


Logger::Logger() :
        _currentStep(0),
        _lastStep(0),
        _directory(FILE_OUTPUT_DIR),
        _log_file_name(FILE_NAME_FOR_GENERAL_OUTPUTS)
{    
    // Create the directory if it doesn't exist
    std::filesystem::create_directories(_directory);

    // Open the log file
    _logFile.open(_directory + _log_file_name, std::ios_base::trunc);

    if (!_logFile.is_open())
    {
        std::cerr << "Unable to create log file: " << _directory + _log_file_name << std::endl;
    }
}

Logger::~Logger()
{
    if (_logFile.is_open())
    {
        _logFile.close();
    }
}

void Logger::setStep(int step)
{
    _currentStep = step;
}

void Logger::log(int k, const std::string& message)
{
    if (!_logFile.is_open())
    {
        std::ostringstream msg;
        msg << "Unable to add to log file in function " << __func__ << "(" << __FILE__ << ")" << std::endl;
        std::cerr << msg.str();
        throw std::runtime_error(msg.str());
    }

    std::ostringstream step_info;
    step_info.str("");
    if (k >= 0)
    {
        _currentStep = k;
        if (_currentStep > _lastStep)
        {
            _logFile << std::endl;
            _lastStep = _currentStep;
        }
        step_info << "Step " << _currentStep << ": ";
    }

    _logFile << step_info.str() << message << std::endl;
}

void Logger::logKalmanFilterDataAfterPrediction(KalmanFilter* filter, const Kalman::Input& u)
{
        // Log the necessary matrices and data vectors
        _x_vehicle_pred.push_back(filter->getVehicleStates());
        _x_vehicle_pred_stdev.push_back(filter->getVehicleStateStdev());
        _u_noisy.push_back(DataVector(u.data(), u.data() + u.size()));
}

void Logger::logKalmanFilterDataAfterUpdate(KalmanFilter* filter)
{
        // Log the necessary matrices and data vectors
        _x_vehicle_est.push_back(filter->getVehicleStates());
        _x_vehicle_est_stdev.push_back(filter->getVehicleStateStdev());
        _innovation.push_back(filter->getInnovation());
        _innovation_cov.push_back(filter->getInnovationCov());
        _z_obs.push_back(filter->getMeasurement());
        _z_pred.push_back(filter->getPredictedMeasurement());
        _chi2.push_back(filter->get_chi2());
}


void Logger::saveAllData()
{
    saveDataToFile(_x_vehicle_pred, generateNewFilename("x_vehicle_pred", FILE_REFPATH));
    saveDataToFile(_x_vehicle_est, generateNewFilename("x_vehicle_est", FILE_REFPATH));
    saveDataToFile(_x_vehicle_pred_stdev, generateNewFilename("x_vehicle_pred_stdev", FILE_REFPATH));
    saveDataToFile(_x_vehicle_est_stdev, generateNewFilename("x_vehicle_est_stdev", FILE_REFPATH));
    saveDataToFile(_u_noisy, generateNewFilename("u_noisy", FILE_REFPATH));
    saveDataToFile(_innovation, generateNewFilename("innovation", FILE_REFPATH));
    saveDataToFile(_innovation_cov, generateNewFilename("innovationCovariance", FILE_REFPATH));
    saveDataToFile(_chi2, generateNewFilename("chi2_innovation_gate", FILE_REFPATH));
    saveDataToFile(_z_obs, generateNewFilename("z_obs", FILE_REFPATH));
    saveDataToFile(_z_pred, generateNewFilename("z_pred", FILE_REFPATH));
}


template <typename VectorType>
void Logger::logVector(int k, const std::string& label, const VectorType& vector)
{
    if (!_logFile.is_open())
    {
        std::ostringstream msg;
        msg << "Unable to add to log file in function " << __func__ << "(" << __FILE__ << ")" << std::endl;
        std::cerr << msg.str();
        throw std::runtime_error(msg.str());
    }

    std::ostringstream step_info;
    step_info.str("");
    if (k >= 0)
    {
        _currentStep = k;
        if (_currentStep > _lastStep)
        {
            _logFile << std::endl;
            _lastStep = _currentStep;
        }
        step_info << "Step " << _currentStep << ": ";
    }

    _logFile << step_info.str() << label << std::endl;
    _logFile << std::scientific << std::setprecision(2);  // Set scientific notation with 2 significant digits
    for (int i = 0; i < vector.size(); ++i)
    {
        _logFile << std::setw(10) << vector(i);  // Right-align with width of 10
        if (i < vector.size() - 1)
        {
            _logFile << " ";  // Space between elements
        }
    }
    _logFile << std::endl;
}

template <typename MatrixType>
void Logger::logMatrix(int k, const std::string& label, const MatrixType& matrix)
{
    if (!_logFile.is_open())
    {
        std::ostringstream msg;
        msg << "Unable to add to log file in function " << __func__ << "(" << __FILE__ << ")" << std::endl;
        std::cerr << msg.str();
        throw std::runtime_error(msg.str());
    }

    std::ostringstream step_info;
    step_info.str("");
    if (k >= 0)
    {
        _currentStep = k;
        if (_currentStep > _lastStep)
        {
            _logFile << std::endl;
            _lastStep = _currentStep;
        }
        step_info << "Step " << _currentStep << ": ";
    }

    _logFile << step_info.str() << label << std::endl;

    _logFile << std::scientific << std::setprecision(2);  // Set scientific notation with 2 significant digits
    for (int i = 0; i < matrix.rows(); ++i)
    {
        _logFile << "         ";  // Indent matrix rows
        for (int j = 0; j < matrix.cols(); ++j)
        {
            _logFile << std::setw(10) << matrix(i, j);  // Right-align with width of 10
            if (j < matrix.cols() - 1)
            {
                _logFile << " ";
            }
        }
        _logFile << std::endl;
    }
}

template <typename MatrixType>
void Logger::logMatrixInMatlabVariableFormat(const std::string& label, const MatrixType& matrix, long int k)
{
    std::ostringstream filename;
    filename << "TO_BE_LOADED_" << label << ".m";

    // "filename" is the name of the matrix set that will be saved in its individual m-file
    // so that it can just be executed. If the Matrix is NxM and there are K steps in the program
    // execution, the resulting matrix will be N x M x K. For dynamic matrix, allocate at maximum
    std::string fullpath = _directory + "/" + filename.str();

    // Check if this file has already been opened in this session
    if (_openedFiles.find(fullpath) == _openedFiles.end())
    {
        // First time opening this file, so open in overwrite mode
        std::ofstream file(fullpath, std::ios::trunc);
        if (file.is_open())
        {
            logMatrixInMatlabVariableFormatSingleOperation(file, label, matrix, k);
            _openedFiles.insert(fullpath);  // Mark this file as opened
        }
        else
        {
            std::ostringstream msg;
            msg << "Unable to open log file in function " << __func__ << "(" << __FILE__ << ")" << std::endl;
            std::cerr << msg.str();
            throw std::runtime_error(msg.str());
        }
    }
    else
    {
        // File has already been opened, so open in append mode
        std::ofstream file(fullpath, std::ios::app);
        if (file.is_open())
        {
            logMatrixInMatlabVariableFormatSingleOperation(file, label, matrix, k);
        }
        else
        {
            std::ostringstream msg;
            msg << "Unable to add to log file in function " << __func__ << "(" << __FILE__ << ")" << std::endl;
            std::cerr << msg.str();
            throw std::runtime_error(msg.str());
        }
    }
}
template <typename MatrixType>
void Logger::logMatrixInMatlabVariableFormatSingleOperation(std::ofstream& file, const std::string& label, const MatrixType& matrix, long int k)
{
    file << label << "(1:" << matrix.rows() << ",1:" << matrix.cols() << "," << k << ") = [\n";
    file << std::scientific << std::setprecision(2);  // Set scientific notation with 2 significant digits

    for (int i = 0; i < matrix.rows(); ++i)
    {
        // Indent matrix rows
        file << "         ";
        for (int j = 0; j < matrix.cols(); ++j)
        {
            // Right-align with width of 10
            file << std::setw(10) << matrix(i, j);
            if (j < matrix.cols() - 1)
            {
                // Add space between elements in a row
                file << " ";
            }
        }
        // End of the row
        file << ";\n";
    }
    // End of the matrix
    file << "];\n\n";
}





// Explicit template instantiation for the types used in KalmanFilter
template void Logger::logMatrix<Eigen::Matrix<double, Eigen::Dynamic, 1>>(int k, const std::string& label, const Eigen::Matrix<double, Eigen::Dynamic, 1>& matrix);
template void Logger::logMatrix<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>(int k, const std::string& label, const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& matrix);
template void Logger::logMatrix<Kalman::Input>(int k, const std::string& label, const Kalman::Input& matrix);
template void Logger::logMatrix<Kalman::ObservationWithTag>(int k, const std::string& label, const Kalman::ObservationWithTag& matrix);
template void Logger::logMatrix<Kalman::ObservationCovariance>(int k, const std::string& label, const Kalman::ObservationCovariance& matrix);
template void Logger::logMatrix<Kalman::FullStateToInovationTransition>(int k, const std::string& label, const Kalman::FullStateToInovationTransition& matrix);
template void Logger::logVector<Eigen::Matrix<double, Eigen::Dynamic, 1>>(int k, const std::string& label, const Eigen::Matrix<double, Eigen::Dynamic, 1>& vector);

template void Logger::logMatrixInMatlabVariableFormat<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>(const std::string& label, const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& matrix, long int k);
template void Logger::logMatrixInMatlabVariableFormat<Kalman::KalmanGain>(const std::string& label, const Kalman::KalmanGain& matrix, long int k);
template void Logger::logMatrixInMatlabVariableFormat<Kalman::InputCovarianceTransfer>(const std::string& label, const Kalman::InputCovarianceTransfer& matrix, long int k);
template void Logger::logMatrixInMatlabVariableFormat<Kalman::VehicleStateCovariance>(const std::string& label, const Kalman::VehicleStateCovariance& matrix, long int k);
template void Logger::logMatrixInMatlabVariableFormat<Kalman::InputCovariance>(const std::string& label, const Kalman::InputCovariance& matrix, long int k);
template void Logger::logMatrixInMatlabVariableFormat<Kalman::VehicleToObservationTransition>(const std::string& label, const Kalman::VehicleToObservationTransition& matrix, long int k);
template void Logger::logMatrixInMatlabVariableFormat<Kalman::LandmarkStateToObservationTransition>(const std::string& label, const Kalman::LandmarkStateToObservationTransition& matrix, long int k);
template void Logger::logMatrixInMatlabVariableFormat<Eigen::Matrix<double, 1, 1>>(const std::string& label, const Eigen::Matrix<double, 1, 1>& matrix, long int k);

template void Logger::logMatrixInMatlabVariableFormatSingleOperation<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>(std::ofstream& file, const std::string& label, const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& matrix, long int k);
template void Logger::logMatrixInMatlabVariableFormatSingleOperation<Kalman::KalmanGain>(std::ofstream& file, const std::string& label, const Kalman::KalmanGain& matrix, long int k);
template void Logger::logMatrixInMatlabVariableFormatSingleOperation<Kalman::InputCovarianceTransfer>(std::ofstream& file, const std::string& label, const Kalman::InputCovarianceTransfer& matrix, long int k);
template void Logger::logMatrixInMatlabVariableFormatSingleOperation<Kalman::VehicleStateCovariance>(std::ofstream& file, const std::string& label, const Kalman::VehicleStateCovariance& matrix, long int k);
template void Logger::logMatrixInMatlabVariableFormatSingleOperation<Kalman::InputCovariance>(std::ofstream& file, const std::string& label, const Kalman::InputCovariance& matrix, long int k);
template void Logger::logMatrixInMatlabVariableFormatSingleOperation<Kalman::VehicleToObservationTransition>(std::ofstream& file, const std::string& label, const Kalman::VehicleToObservationTransition& matrix, long int k);
template void Logger::logMatrixInMatlabVariableFormatSingleOperation<Kalman::LandmarkStateToObservationTransition>(std::ofstream& file, const std::string& label, const Kalman::LandmarkStateToObservationTransition& matrix, long int k);
template void Logger::logMatrixInMatlabVariableFormatSingleOperation<Eigen::Matrix<double, 1, 1>>(std::ofstream& file, const std::string& label, const Eigen::Matrix<double, 1, 1>& matrix, long int k);


