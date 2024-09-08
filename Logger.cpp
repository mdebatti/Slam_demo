#include "Logger.h"
#include <Eigen/Dense>
#include <sstream>
#include <iomanip>  // For std::setw and std::setprecision

Logger::Logger(const std::string& filename) : _currentStep(0),
    _lastStep(0)
{
    // Define the relative path to the outputs directory
    std::string outputDir = "../cpp/outputs/";

    // Create the directory if it doesn't exist
    std::filesystem::create_directories(outputDir);

    // Open the log file in append mode
    _logFile.open(outputDir + filename, std::ios_base::trunc);

    if (!_logFile.is_open())
    {
        std::cerr << "Unable to create log file: " << outputDir + filename << std::endl;
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
        std::cerr << "Unable to add to log file in function " << __func__ << "(" << __FILE__ << ")" << std::endl;
        std::exit(EXIT_FAILURE);
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

template <typename VectorType>
void Logger::logVector(int k, const std::string& label, const VectorType& vector)
{
    if (!_logFile.is_open())
    {
        std::cerr << "Unable to add to log file in function " << __func__ << "(" << __FILE__ << ")" << std::endl;
        std::exit(EXIT_FAILURE);
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
        std::cerr << "Unable to add to log file in function " << __func__ << "(" << __FILE__ << ")" << std::endl;
        std::exit(EXIT_FAILURE);
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

// Explicit template instantiation for the types used in KalmanFilter
template void Logger::logMatrix<Eigen::Matrix<double, Eigen::Dynamic, 1>>(int k, const std::string& label, const Eigen::Matrix<double, Eigen::Dynamic, 1>& matrix);
template void Logger::logMatrix<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>(int k, const std::string& label, const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& matrix);
template void Logger::logMatrix<Kalman::Input>(int k, const std::string& label, const Kalman::Input& matrix);
template void Logger::logMatrix<Kalman::ObservationWithTag>(int k, const std::string& label, const Kalman::ObservationWithTag& matrix);
template void Logger::logMatrix<Kalman::ObservationCovariance>(int k, const std::string& label, const Kalman::ObservationCovariance& matrix);
template void Logger::logMatrix<Kalman::FullStateToInovationTransition>(int k, const std::string& label, const Kalman::FullStateToInovationTransition& matrix);
template void Logger::logVector<Eigen::Matrix<double, Eigen::Dynamic, 1>>(int k, const std::string& label, const Eigen::Matrix<double, Eigen::Dynamic, 1>& vector);


