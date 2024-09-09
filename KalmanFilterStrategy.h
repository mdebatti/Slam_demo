#pragma once

#include "types.h"

class KalmanFilterStrategy {
public:
    virtual void predict(const Kalman::Input& u, double n) = 0;
    virtual void update() = 0;
    virtual void addState() = 0;
    virtual bool isMapped(const Kalman::ObservationWithTag& z) = 0;
    virtual ~KalmanFilterStrategy() = default;
};
