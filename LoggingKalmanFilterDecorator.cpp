#include "LoggingKalmanFilterDecorator.h"

LoggingKalmanFilterDecorator::LoggingKalmanFilterDecorator(std::unique_ptr<KalmanFilterStrategy> wrapped, Logger& logger)
    : _wrapped(std::move(wrapped)),
      _logger(logger)
{
        //std::cout << "Type of _wrapped: " << typeid(*_wrapped.get()).name() << std::endl;
}


void LoggingKalmanFilterDecorator::predict(const Kalman::Input& u, double wheel_radius_noise)
{
    // increment the time step counter (only used for logging)
    ++_timeStep;

    _wrapped->predict(u, wheel_radius_noise);

    KalmanFilter* kf = dynamic_cast<KalmanFilter*>(_wrapped.get());
    if (kf)
    {
        Eigen::Matrix<double, 1, 1> predict_phi;
        predict_phi(0, 0) = u(1) + kf->get_x()(2);

        logMatrixInMatlabVariableFormat("predict_phi", predict_phi, _timeStep);
        logMatrixInMatlabVariableFormat("predict_X", kf->get_X(), _timeStep);
        logMatrixInMatlabVariableFormat("predict_G", kf->get_G(), _timeStep);
        logMatrixInMatlabVariableFormat("predict_F", kf->get_F(), _timeStep);
        logMatrixInMatlabVariableFormat("predict_sigma_u", kf->get_sigma_u(), _timeStep);
        kf->logControlInput(_timeStep,u);
        logKalmanFilterDataAfterPrediction(u);
    }
    else
    {
        std::cerr << "Error: dynamic_cast to KalmanFilter failed in predict()." << std::endl;
        throw std::runtime_error("dynamic_cast to KalmanFilter failed in predict().");
    }
}

void LoggingKalmanFilterDecorator::update()
{
    _wrapped->update();

    KalmanFilter* kf = dynamic_cast<KalmanFilter*>(_wrapped.get());
    if (kf)
    {
        logMatrixInMatlabVariableFormat("update_X", kf->get_X(), _timeStep);
        kf->logObservation(_timeStep, kf->get_z());
        kf->logInnovationAndCovariance(_timeStep,"Innovation vector and its covariance after update");
        kf->logStateToInnovationTransferMatrixH(_timeStep,"State Covariance to Innovation covariance transfer matrix H");
        logMatrixInMatlabVariableFormat("update_Hv", kf->get_Hv(), _timeStep);
        logMatrixInMatlabVariableFormat("update_Hp", kf->get_Hp(), _timeStep);
        logMatrixInMatlabVariableFormat("update_sigma_z", kf->get_sigma_z(), _timeStep);
        logMatrixInMatlabVariableFormat("KalmanGain", kf->get_K(), _timeStep);
        logMatrixInMatlabVariableFormat("InnovationCovariance", kf->get_S(), _timeStep);

        // After measurement is taken, we either end up here or in addState - log after each measurement
        logKalmanFilterDataAfterUpdate();

        // These specifically get updated in addState
        logMatrixInMatlabVariableFormat("addstate_Tx", kf->get_Tx(), _timeStep);
        logMatrixInMatlabVariableFormat("addstate_Tz", kf->get_Tz(), _timeStep);
        logMatrixInMatlabVariableFormat("addstate_mu", kf->get_mu(), _timeStep);
        logMatrixInMatlabVariableFormat("addstate_sigma", kf->get_sigma_new_landmark(), _timeStep);
    }
    else
    {
        std::cerr << "Error: dynamic_cast to KalmanFilter failed in update()." << std::endl;
        throw std::runtime_error("dynamic_cast to KalmanFilter failed in update().");
    }
}

bool LoggingKalmanFilterDecorator::addState()
{
    bool state_added = _wrapped->addState();
    KalmanFilter* kf = dynamic_cast<KalmanFilter*>(_wrapped.get());
    if(state_added)
    {
        if (kf)
        {
            std::cout << "Time step " << _timeStep << ": Added beacon #" << int(kf->get_z()(2)) << " (" << kf->get_obsWRF()(0) << "m, " << kf->get_obsWRF()(1) << "m) to the map" << endl;
            kf->displayStateCovarianceMatrix();
        }
        else
        {
            std::cerr << "Error: dynamic_cast to KalmanFilter failed in addState()." << std::endl;
            throw std::runtime_error("dynamic_cast to KalmanFilter failed in addState().");
        }
    }

    // For logging at each time step k
    if (kf)
    {

        kf->logObservation(_timeStep,kf->get_z());
        kf->logInnovationAndCovariance(_timeStep,"Innovation vector and its covariance after update");
        kf->logStateToInnovationTransferMatrixH(_timeStep,"State Covariance to Innovation covariance transfer matrix H");

        // After measurement is taken, we either end up here or in addState - log after each measurement
        logKalmanFilterDataAfterUpdate();

        logMatrixInMatlabVariableFormat("addState_X", kf->get_X(), _timeStep);
        logMatrixInMatlabVariableFormat("KalmanGain", kf->get_K(), _timeStep);
        logMatrixInMatlabVariableFormat("InnovationCovariance", kf->get_S(), _timeStep);

        // These specifically get updated in addState
        logMatrixInMatlabVariableFormat("addstate_Tx", kf->get_Tx(), _timeStep);
        logMatrixInMatlabVariableFormat("addstate_Tz", kf->get_Tz(), _timeStep);
        logMatrixInMatlabVariableFormat("addstate_mu", kf->get_mu(), _timeStep);
        logMatrixInMatlabVariableFormat("addstate_sigma", kf->get_sigma_new_landmark(), _timeStep);
    }
    else
    {
        std::cerr << "Error: dynamic_cast to KalmanFilter failed in addState()." << std::endl;
        throw std::runtime_error("dynamic_cast to KalmanFilter failed in addState().");
    }

    return state_added;
}

bool LoggingKalmanFilterDecorator::isMapped(const Kalman::ObservationWithTag& z)
{
    KalmanFilter* kf = dynamic_cast<KalmanFilter*>(_wrapped.get());
    if (kf)
    {
        kf->reset_K();
        kf->reset_Hp();
        kf->reset_Hv();
        kf->reset_sigma_z();
        kf->reset_Tx();
        kf->reset_Tz();
        kf->reset_mu();
        kf->reset_sigma_new_landmark();
        kf->reset_S();
    }
    else
    {
        std::cerr << "Error: dynamic_cast to KalmanFilter failed in addState()." << std::endl;
        throw std::runtime_error("dynamic_cast to KalmanFilter failed in addState().");
    }
    return _wrapped->isMapped(z);
}

 template <typename MatrixType>
void LoggingKalmanFilterDecorator::logMatrixInMatlabVariableFormat(const std::string& label, const MatrixType& matrix, long int k) const
{
    _logger.logMatrixInMatlabVariableFormat(label, matrix, k);
}

void LoggingKalmanFilterDecorator::logKalmanFilterDataAfterUpdate()
{
    KalmanFilter* concrete_filter = dynamic_cast<KalmanFilter*>(_wrapped.get());
    if (concrete_filter)
    {
        _logger.logKalmanFilterDataAfterUpdate(concrete_filter);
    }
    else
    {
        std::cerr << "Error: dynamic_cast to KalmanFilter failed in logData()." << std::endl;
        throw std::runtime_error("dynamic_cast to KalmanFilter failed in logData().");
    }
}

void LoggingKalmanFilterDecorator::logKalmanFilterDataAfterPrediction(const Kalman::Input& u)
{
    KalmanFilter* concrete_filter = dynamic_cast<KalmanFilter*>(_wrapped.get());
    if (concrete_filter)
    {
        _logger.logKalmanFilterDataAfterPrediction(concrete_filter, u);
    }
    else
    {
        std::cerr << "Error: dynamic_cast to KalmanFilter failed in logData()." << std::endl;
        throw std::runtime_error("dynamic_cast to KalmanFilter failed in logData().");
    }
}
