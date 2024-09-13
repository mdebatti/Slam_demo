#pragma once
#include "KalmanFilterStrategy.h"
#include "KalmanFilter.h"

class LoggingKalmanFilterDecorator : public KalmanFilterStrategy
{
public:
    LoggingKalmanFilterDecorator(std::unique_ptr<KalmanFilterStrategy> wrapped, Logger& logger);

    void predict(const Kalman::Input& u, double wheel_radius_noise) override;
    void update() override;
    bool addState() override;
    bool isMapped(const Kalman::ObservationWithTag& z) override;


private:
    std::unique_ptr<KalmanFilterStrategy> _wrapped;
    Logger& _logger;


    // Time step counter
    long int _timeStep = 0;

    template <typename MatrixType>
    void logMatrixInMatlabVariableFormat(const std::string& label, const MatrixType& matrix, long int k) const;

    void logKalmanFilterDataAfterUpdate();

    void logKalmanFilterDataAfterPrediction(const Kalman::Input& u);
};

