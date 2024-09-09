#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(Logger& logger,
                           const Kalman::VehicleState& x_init,
                           const Kalman::VehicleState& diag_X_init,
                           double var_q,
                           double var_w,
                           double var_s,
                           double var_g,
                           double var_r,
                           double var_range,
                           double var_bearing)
      : _logger(logger),
        _x(x_init),
        _var_q(var_q),
        _var_w(var_w),
        _var_s(var_s),
        _var_g(var_g),
        _var_r(var_r),
        _var_range(var_range),
        _var_bearing(var_bearing)
{
    // Initialise the state covariance matrix.
    // It only contains the 4 vehicle states at the begining (i.e: no landmarks)
     _X.diagonal() = diag_X_init;

     _logger.log(-1, "KalmanFilter initialized.");
     logStateAndCovariance(-1, "Initial state and covariance");

     std::cout << "Kalman filter initialisation: " <<  std::endl;
     std::cout << "State initialisation (X, Y, orientation, wheel radius): " << _x.transpose() << std::endl;
     std::cout << "State covariance matrix diagonal values: " << _X.diagonal().transpose() << std::endl;
}


void KalmanFilter::logStateAndCovariance(int k, const std::string& message) const
{
    _logger.logMatrix(k, message + " - State (_x)", _x);
    _logger.logMatrix(k, message + " - Covariance (_X)", _X);
}

void KalmanFilter::logInnovationAndCovariance(int k, const std::string& message) const
{
    // Only log if there has been a feature matched
    if (_z(2) >=0)
    {
        _logger.logMatrix(k, message + " - Innovation (_v)", _v);
        _logger.logMatrix(k, message + " - Covariance (_S)", _S);
    }
}

void KalmanFilter::logStateToInnovationTransferMatrixH(int k,  const std::string& message) const
{
    // Only log if there has been a feature matched
    if (_z(2) >=0)
    {
        _logger.logMatrix(k, message + " - Covariance (_H)", _H);
    }
}

void KalmanFilter::logControlInput(int k, const Kalman::Input& u) const
{
    _logger.logMatrix(k, "Control input into prediction step - input (u)", u);
}

void KalmanFilter::logObservation(int k, const Kalman::ObservationWithTag& z) const
{
    _logger.logMatrix(k, "Observation before update step - (z)", z);
}

void KalmanFilter::log(int k, const std::string& message) const
{
    _logger.log(k, message);
}

// Function to generate a one-step vehicle prediction from previous estimate and control input.
void KalmanFilter::predict(const Kalman::Input& u, double wheel_radius_noise)
{
    // x(k+1|k) is based on the process model and knowledge of the control input u(k)
    // For the first step, there is no uncertainty
    // and state prediction should be the initial condition
    // x(k) = [X(k) Y(k) Theta(k) wheel_radius_deformed(k) Xb1 Yb1 ... XbN YbN]'
    // u(k) = [wheel_angular_velocity(k), wheel_steer_angle(k), wheel_radius_noise]'

    // angle of front wheel in the global reference frame
    double phi = _x(2) + u(1);

    // Source error transfer matrix
    _G(0,0) =  _dt*cos(phi);
    _G(0,1) = -_dt*sin(phi);
    _G(1,0) =  _dt*sin(phi);
    _G(1,1) =  _dt*cos(phi);
    _G(2,0) =  _dt*sin(u(1)) / SLAM_PHYSICAL_CONST::WHEEL_BASE;
    _G(2,1) =  _dt*cos(u(1)) / SLAM_PHYSICAL_CONST::WHEEL_BASE;
    _G(3,2) =  _dt;

    // _x(3,0) is the wheel radius, add random noise wheel_radius_noise to model elastic deformation
    double wheel_radius_deformed = _x(3,0) + wheel_radius_noise;

    // state transition matrix F
    _F(0,0) =  1.0;
    _F(0,2) = -_dt*wheel_radius_deformed*u(0)*sin(phi);
    _F(0,3) =  _dt*u(0)*cos(phi);

    _F(1,1) =  1.0;
    _F(1,2) =  _dt*wheel_radius_deformed*u(0)*cos(phi);
    _F(1,3) =  _dt*u(0)*sin( phi );

    _F(2,2) =  1.0;
    _F(2,3) =  _dt*u(0)*sin(u(1)) / SLAM_PHYSICAL_CONST::WHEEL_BASE;

    _F(3,3) =  1.0;

    // source error covariance
    _sigma_u(0,0) = _var_q*(wheel_radius_deformed*u(0))*(wheel_radius_deformed*u(0)) + _var_w*wheel_radius_deformed*wheel_radius_deformed;
    _sigma_u(1,1) = _var_s*(wheel_radius_deformed*u(0)*u(1))*(wheel_radius_deformed*u(0)*u(1)) + _var_g*(wheel_radius_deformed*u(0))*(wheel_radius_deformed*u(0));
    _sigma_u(2,2) = _var_r;

    // state prediction
    _x(0) = _x(0) + _dt*wheel_radius_deformed*u(0)*cos( phi );
    _x(1) = _x(1) + _dt*wheel_radius_deformed*u(0)*sin( phi );
    _x(2) = _x(2) + _dt*wheel_radius_deformed*u(0)*sin(u(1)) / SLAM_PHYSICAL_CONST::WHEEL_BASE;
    _x(3) = wheel_radius_deformed;

    // Prediction cross covariance for the vehicle - writing/reading to/from X by reference
    Kalman::XVehicleBlock(_X) = _F * Kalman::XVehicleBlock(_X) * _F.transpose() + _G * _sigma_u * _G.transpose();

    // Can only modify the parts of _X corresponding to the vehicle - landmark covariance
    // if there are at least 1 landmark in the state covariance matrix
    if (_X.rows() > SLAM_ARRAY_SIZE::VEHICLE_STATE_DIM)
    {
        // Prediction cross covariance between the vehicle and landmarks - writing/reading to/from X by reference
        Kalman::LandmarkToVehicleCrossCovariance(_X) = Kalman::LandmarkToVehicleCrossCovariance(_X) * _F.transpose();

        // Prediction cross covariance between the landmarks and the vehicle - writing/reading to/from X by reference
        Kalman::VehicleToLandmarkCrossCovariance(_X) = _F.transpose() * Kalman::VehicleToLandmarkCrossCovariance(_X);
    }

    // The prediction covariance of the landmarks does not change in this step!

    // For all subsequent calls, add noise by setting the sampling period to its actuall value
    _dt = SLAM_CONST::DT;
}

void KalmanFilter::update()
{
    // important, _z is set in isMapped(z), which must always be called before addState (modify code otherwise)

    // Subscripts for accessing observed landmark state and Covariance.
    // landmark tags (in obsRB(2)) start at 0 for first landmark
    int px = SLAM_ARRAY_SIZE::VEHICLE_STATE_DIM + SLAM_ARRAY_SIZE::OBSERVATION_DIM*_z(2);

    // X and Y offset between observed landmark and radar origin
    double dx = _x(px) - (_x(0) + SLAM_PHYSICAL_CONST::XOFFSET*cos(_x(2)) - SLAM_PHYSICAL_CONST::YOFFSET*sin(_x(2)));
    double dy = _x(px+1) - (_x(1) + SLAM_PHYSICAL_CONST::XOFFSET*sin(_x(2)) + SLAM_PHYSICAL_CONST::YOFFSET*cos(_x(2))) ;

    // Range between observed landmark and radar origin
    double r2 = dx*dx + dy*dy;
    double r  = sqrt(r2);

    const double epsilon = 1e-3;
    if (r < epsilon) {
        std::cerr << "Warning: r is too small, potential numerical instability." << std::endl;
        r = epsilon;
    }

    // Jacobian of the prediction model with respect to the vehicle state
    double Dxo = SLAM_PHYSICAL_CONST::XOFFSET*cos(_x(2)) - SLAM_PHYSICAL_CONST::YOFFSET*sin(_x(2));
    double Dyo = SLAM_PHYSICAL_CONST::XOFFSET*sin(_x(2)) + SLAM_PHYSICAL_CONST::YOFFSET*cos(_x(2));

    _Hv(0,0) = -dx/r;
    _Hv(0,1) = -dy/r;
    _Hv(0,2) = (dx*Dyo - dy*Dxo)/r;
    _Hv(1,0) = dy/r2;
    _Hv(1,1) = -dx/r2;
    _Hv(1,2) = -((dx*Dxo + dy*Dyo )/r2 + 1);

    // Jacobian of the prediction model with respect to the landmark location
    _Hp(0,0) = dx/r;
    _Hp(0,1) = dy/r;
    _Hp(1,0) = -dy/r2;
    _Hp(1,1) = dx/r2;

    // get orientation between 0 and 360deg
    double phi = normalizeAngle(_x(2));

    // observation model
    _zpred(0) = r;
    _zpred(1) = normalizeAngle(atan2(dy,dx) - phi);  //normalisation

    // observation z and its covariance matrix exprimed in range and bearing
    _sigma_z(0,0) = _var_range;
    _sigma_z(1,1) = _var_bearing;

    _v(0) = _z(0) - _zpred(0);
    _v(1) = normalizeAngle(_z(1) - _zpred(1));

    // 2 x numStates to transfer full state covariance to innovation covariance (2 x 2)
    _H.setZero();

    // Direct assignment to the blocks in matrix H
    _H.block<SLAM_ARRAY_SIZE::OBSERVATION_DIM, SLAM_ARRAY_SIZE::VEHICLE_STATE_DIM>(0, 0) = _Hv;
    _H.block<SLAM_ARRAY_SIZE::OBSERVATION_DIM, SLAM_ARRAY_SIZE::OBSERVATION_DIM>(0, px) = _Hp;

    //Innovation update, kalman gain, state and covariance innovation
    _S  = _H*_X*_H.transpose() + _sigma_z;

    // Check for NaNs or Infs
    if (_S.hasNaN() || _S.determinant() == 0.0)
    {
        std::cerr << "Warning: _S contains NaN or is singular." << std::endl;
    }
    if (_v.hasNaN())
    {
        std::cerr << "Warning: _v contains NaN." << std::endl;
    }

    // Compute chi2
    _chi2 = _v.transpose() * _S.inverse() * _v;

    if ( _chi2 < 7.5)
    {
        // Kalman Gain (numStates x 2 )
        Kalman::KalmanGain K = _X*_H.transpose()*_S.inverse();

        // Update of the state vector and its covariance matrix
        _x  = _x + K*_v;
        _X  = _X - K*_S*K.transpose();
    }
    else
    {
        //std::cout << "Innovation gate failed" << endl;
    }
}

void KalmanFilter::addState()
{
    // important, _z is set in isMapped(z), which must always be called before addState (modify code otherwise)

    // early return if the matching index was -1 (no observation on this step)
    if (_z(2) < 0.0)
    {
        return;
    }

    // bearing of observed landmark in global reference frame
    double phi = _x(2) + _z(1);

    // X and Y distance from radar origin to landmark
    double dxo = _z(0)*cos(phi);
    double dyo = _z(0)*sin(phi);

    // Offset from vehicle reference frame to radar reference frame
    double dxr = SLAM_PHYSICAL_CONST::XOFFSET*cos(_x(2)) - SLAM_PHYSICAL_CONST::YOFFSET*sin(_x(2));
    double dyr = SLAM_PHYSICAL_CONST::XOFFSET*sin(_x(2)) + SLAM_PHYSICAL_CONST::YOFFSET*cos(_x(2));

    // Transformation matrix (2 x 4) - Matrix initialised at 0 at creation
    _Tx(0,0) = 1.0;
    _Tx(0,2) = -(dyr + dyo);
    _Tx(1,1) = 1.0;
    _Tx(1,2) = dxr + dxo;

    // Transformation matrix (2 x 2)
    _Tz(0,0) = cos(phi);
    _Tz(0,1) = -dyo;
    _Tz(1,0) = sin(phi);
    _Tz(1,1) = dxo;

    // Noise covariance matrix of measurement in (range, bearing)
    _R(0,0) = _var_range;
    _R(1,1) = _var_bearing;

    // Calculate mu (cross-covariance of new landmark with existing state), a N x OBSERVATION_DIM matrix
    Eigen::MatrixXd mu = Kalman::XVehicleAndStateBlockTallAndSkinny(_X) * _Tx.transpose();

    // Calculate Sigma (covariance of new landmark), a OBSERVATION_DIM x OBSERVATION_DIM matrix
    Eigen::Matrix2d Sigma = _Tx * Kalman::XVehicleBlock(_X) * _Tx.transpose() + _Tz * _R * _Tz.transpose();

    // z is expressed in range and bearing. transform it to a WRF (global X, Y)
    _obsWRF(0) = _x(0) + dxr + dxo;
    _obsWRF(1) = _x(1) + dyr + dyo;

    // Resize the state vector x and state covariance X
    int newStateStart = _x.rows();
    int requiredRows = newStateStart + _obsWRF.rows();
    if (newStateStart < requiredRows)
    {
        _x.conservativeResize(requiredRows, _x.cols());
        _x.block(newStateStart, 0, _obsWRF.rows(), _x.cols()) = _obsWRF;

        // Update X with mu and Sigma
        _X.conservativeResize(requiredRows, requiredRows);

        // Set the bottom-right corner with Sigma
        _X.bottomRightCorner(Sigma.rows(), Sigma.cols()) = Sigma;

        // Set the top-right and bottom-left corners with mu
        _X.topRightCorner(mu.rows(), mu.cols()) = mu;                   // This ensures correct dimensions
        _X.bottomLeftCorner(mu.cols(), mu.rows()) = mu.transpose();

        // resize the state to innovation transfer matrix
        _H.resize(SLAM_ARRAY_SIZE::OBSERVATION_DIM, _x.rows());
    }

    // Augment the mapped andmark list
    std::cout << "Added beacon #" << int(_z(2)) << " (" << _obsWRF(0) << "m, " << _obsWRF(1) << ") to the map" << endl;
    displayStateCovarianceMatrix();

    _mappedLandmarkList.push_back( _z(2));
}

void KalmanFilter::displayStateCovarianceMatrix() const
{
    std::cout << "State covariance matrix _X:" << std::endl;

    for (int i = 0; i < _X.rows(); ++i) {
        for (int j = 0; j < _X.cols(); ++j) {
            std::cout << std::setw(10) << std::setprecision(4) << _X(i, j) << " ";
        }
        std::cout << std::endl;  // New line at the end of each row
    }
    std::cout << std::endl;
}

void KalmanFilter::displayHMatrix(const Kalman::FullStateToInovationTransition& H) const
{
    std::cout << "Observation to state transfer matrix H:" << std::endl;

    for (int i = 0; i < H.rows(); ++i) {
        for (int j = 0; j < H.cols(); ++j) {
            std::cout << std::setw(10) << std::setprecision(4) << H(i, j) << " ";
        }
        std::cout << std::endl;  // New line at the end of each row
    }
    std::cout << std::endl;
}

void KalmanFilter::displayVehicleStateToInnovationTransferMatrix() const
{
    std::cout << "Vehicle state covariance to innovation covariance transfer matrix _Hv:" << std::endl;

    for (int i = 0; i < _Hv.rows(); ++i) {
        for (int j = 0; j < _Hv.cols(); ++j) {
            std::cout << std::setw(10) << std::setprecision(4) << _Hv(i, j) << " ";
        }
        std::cout << std::endl;  // New line at the end of each row
    }
    std::cout << std::endl;
}

void KalmanFilter::displaylandmarkStateToInnovationTransferMatrix() const
{
    std::cout << "Landmark state covariance to innovation covariance transfer matrix Hp:" << std::endl;

    for (int i = 0; i < _Hp.rows(); ++i) {
        for (int j = 0; j < _Hp.cols(); ++j) {
            std::cout << std::setw(10) << std::setprecision(4) << _Hp(i, j) << " ";
        }
        std::cout << std::endl;  // New line at the end of each row
    }
    std::cout << std::endl;
}

bool KalmanFilter::isMapped(const Kalman::ObservationWithTag& obsRB)
{
    // important, _z must be set here! Otherwise modify update() and addState() functions
    _z = obsRB;

    int numMapped = _mappedLandmarkList.size();
    bool result = false;

    for(int i = 0; i < numMapped; ++i)
    {
        if(isEqualWithTolerance(_mappedLandmarkList[i], obsRB(2)))
        {
            //std::cout << "Matched an observation with beacon #" << int(obsRB(2)) << " to the map" << endl;
            result = true;
        }
    }
    return result;
}

DataVector KalmanFilter::eigenToStdVector(const Eigen::VectorXd& eigenVec) const
{
    return DataVector(eigenVec.data(), eigenVec.data() + eigenVec.size());
}

DataVector KalmanFilter::getVehicleStates() const
{
    return eigenToStdVector(_x.head(SLAM_ARRAY_SIZE::VEHICLE_STATE_DIM));
};

DataVector KalmanFilter::getInnovation() const
{
    return eigenToStdVector(_v.head(SLAM_ARRAY_SIZE::OBSERVATION_DIM));
};

DataVector KalmanFilter::getMeasurement() const
{
    return eigenToStdVector(_z.head(SLAM_ARRAY_SIZE::OBSERVATION_SIM_DIM));
};

DataVector KalmanFilter::getPredictedMeasurement() const
{
    return eigenToStdVector(_zpred.head(SLAM_ARRAY_SIZE::OBSERVATION_DIM));
};

double KalmanFilter::get_chi2() const
{
    return _chi2;
};


DataVector KalmanFilter::getInnovationCov(const Kalman::ObservationWithTag& z) const
{
    // Extract the diagonal elements of the matrix _S
    Eigen::VectorXd diagonalElements = _S.diagonal();

    // if there was no match
    if (z(2) < 0)
    {
        diagonalElements.setZero();
    }

    // Convert the Eigen vector to a std::vector<double>
    return eigenToStdVector(diagonalElements);
};

