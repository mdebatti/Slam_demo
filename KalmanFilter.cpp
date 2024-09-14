#include "KalmanFilter.h"
#include "Logger.h"

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
    // First check if matrix multiplication operation match Matlab results
    testMatrixOperationsInPredict();
    testMatrixOperationsInUpdate(x_init);
    testMatrixOperationsInAddState(x_init);

    // RESET STATE OF MATRIXES - CHECK IF NEEDED

    // Initialise the state covariance matrix - It only contains the 4 vehicle states at the begining (i.e: no landmarks)
    _X.diagonal() = diag_X_init;

    //Initialise Kalman gain with 0
    _K.resize(SLAM_ARRAY_SIZE::VEHICLE_STATE_DIM, SLAM_ARRAY_SIZE::OBSERVATION_DIM);
    _K.setZero();

     _logger.log(-1, "KalmanFilter initialized.");
     logStateAndCovariance(-1, "Initial state and covariance");

     std::cout << "Kalman filter initialisation: " <<  std::endl;
     std::cout << "State initialisation (X, Y, orientation, wheel radius): " << _x.transpose() << std::endl;
     std::cout << "State covariance matrix diagonal values: " << _X.diagonal().transpose() << std::endl;
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

    // Only propagate noise if vehicle moving
    if (u(0) > 0.0)
    {
        _dt = SLAM_CONST::DT;
    }

    // Source error transfer matrix G (made index 0-based)
    // G = dt * [cos(x_est(2)+u(1)) -sin(x_est(2)+u(1)) 0;
    //           sin(x_est(2)+u(1))  cos(x_est(2)+u(1)) 0;
    //           sin(u(1))/B         cos(u(1))/B        0;
    //           0                   0                  1];
    _G(0,0) =  _dt*cos(phi);
    _G(0,1) = -_dt*sin(phi);
    _G(1,0) =  _dt*sin(phi);
    _G(1,1) =  _dt*cos(phi);
    _G(2,0) =  _dt*sin(u(1)) / SLAM_PHYSICAL_CONST::WHEEL_BASE;
    _G(2,1) =  _dt*cos(u(1)) / SLAM_PHYSICAL_CONST::WHEEL_BASE;
    _G(3,2) =  _dt;

    // _x(3,0) is the wheel radius, add random noise wheel_radius_noise to model elastic deformation
    double wheel_radius_deformed = _x(3,0) + wheel_radius_noise;

    // state transition matrix F (made index 0-based)
    //
    //  F=[1 0 -dt*R*u(0)*sin(x_est(2)+u(1)) dt*u(0)*cos(x_est(2)+u(1));
    //     0 1  dt*R*u(0)*cos(x_est(2)+u(1)) dt*u(0)*sin(x_est(2)+u(1));
    //     0 0         1                            dt*u(0)*sin(u(1))/B;
    //     0 0         0                                 1          ];
    _F(0,0) =  1.0;
    _F(0,2) = -_dt*wheel_radius_deformed*u(0)*sin(phi);
    _F(0,3) =  _dt*u(0)*cos(phi);

    _F(1,1) =  1.0;
    _F(1,2) =  _dt*wheel_radius_deformed*u(0)*cos(phi);
    _F(1,3) =  _dt*u(0)*sin( phi );

    _F(2,2) =  1.0;
    _F(2,3) =  _dt*u(0)*sin(u(1)) / SLAM_PHYSICAL_CONST::WHEEL_BASE;

    _F(3,3) =  1.0;

    // source error covariance (made index 0-based)
    // sigma=[(var_q*(R*u(0))^2)+(var_w*(R)^2)  0                                           0;
    //        0                                 (var_s*(R*u(0)*u(1))^2)+(var_g*(R*u(0))^2)  0;
    //        0                                 0                                           var_r];
    _sigma_u(0,0) = _var_q*(wheel_radius_deformed*u(0))*(wheel_radius_deformed*u(0)) + _var_w*wheel_radius_deformed*wheel_radius_deformed;
    _sigma_u(1,1) = _var_s*(wheel_radius_deformed*u(0)*u(1))*(wheel_radius_deformed*u(0)*u(1)) + _var_g*(wheel_radius_deformed*u(0))*(wheel_radius_deformed*u(0));
    _sigma_u(2,2) = _var_r;

    // state prediction
    _x(0) = _x(0) + _dt*wheel_radius_deformed*u(0)*cos(phi);
    _x(1) = _x(1) + _dt*wheel_radius_deformed*u(0)*sin(phi);
    _x(2) = _x(2) + _dt*wheel_radius_deformed*u(0)*sin(u(1)) / SLAM_PHYSICAL_CONST::WHEEL_BASE;
    _x(3) = wheel_radius_deformed;

    // Update State covariance matrix
    predictMatrixOperationsDo();
}

void KalmanFilter::predictMatrixOperationsDo()
{
    // Prediction cross covariance for the vehicle - writing/reading to/from X by reference
    Kalman::XVehicleBlock(_X) = _F * Kalman::XVehicleBlock(_X) * _F.transpose() + _G * _sigma_u * _G.transpose();

    // Can only modify the parts of _X corresponding to the vehicle - landmark covariance
    // if there are at least 1 landmark in the state covariance matrix
    if (_X.rows() > SLAM_ARRAY_SIZE::VEHICLE_STATE_DIM)
    {
        // Prediction cross covariance between the vehicle and landmarks - writing/reading to/from X by reference
        // that is the left-sub-corner of _X (NUM_LANDMARK_STATES x NUM_VEHICLE_STATE)
        Kalman::LeftSubCornerPredictionCrossCovariance(_X) = Kalman::LeftSubCornerPredictionCrossCovariance(_X) * _F.transpose();

        // Prediction cross covariance between the landmarks and the vehicle - writing/reading to/from X by reference
        // that is the right-sup-corner of _X (NUM_VEHICLE_STATE x NUM_LANDMARK_STATES)
        Kalman::RightSupCornerPredictionCrossCovariance(_X) = _F * Kalman::RightSupCornerPredictionCrossCovariance(_X);
    }

    // The prediction covariance of the landmarks does not change in this step!
}



void KalmanFilter::update()
{
    // important, _z is set in isMapped(z), which must always be called before addState (modify code otherwise)

    // Subscripts for accessing observed landmark state and Covariance.
    // landmark tags (in obsRB(2)) start at 0 for first landmark
    _px = SLAM_ARRAY_SIZE::VEHICLE_STATE_DIM + SLAM_ARRAY_SIZE::OBSERVATION_DIM*_z(2);

    // X and Y offset between observed landmark and radar origin
    double dx = _x(_px) - (_x(0) + SLAM_PHYSICAL_CONST::XOFFSET*cos(_x(2)) - SLAM_PHYSICAL_CONST::YOFFSET*sin(_x(2)));
    double dy = _x(_px+1) - (_x(1) + SLAM_PHYSICAL_CONST::XOFFSET*sin(_x(2)) + SLAM_PHYSICAL_CONST::YOFFSET*cos(_x(2)));

    // Range between observed landmark and radar origin
    double r2 = dx*dx + dy*dy;
    double r  = sqrt(r2);

    const double epsilon = 1e-3;
    if (r < epsilon)
    {
        std::cerr << "Warning: r is too small, potential numerical instability." << std::endl;
        r = epsilon;
    }

    double Dxo = SLAM_PHYSICAL_CONST::XOFFSET*cos(_x(2)) - SLAM_PHYSICAL_CONST::YOFFSET*sin(_x(2));
    double Dyo = SLAM_PHYSICAL_CONST::XOFFSET*sin(_x(2)) + SLAM_PHYSICAL_CONST::YOFFSET*cos(_x(2));

    // Jacobian of the prediction model with respect to the vehicle state
    // Hv  = [ -dx/r   -dy/r      (dx*Dyo - dy*Dxo)/r        0;
    //          dy/r2  -dx/r2 -((dx*Dxo + dy*Dyo )/r2 + 1)  0];
    _Hv(0,0) = -dx/r;
    _Hv(0,1) = -dy/r;
    _Hv(0,2) = (dx*Dyo - dy*Dxo)/r;
    _Hv(1,0) = dy/r2;
    _Hv(1,1) = -dx/r2;
    _Hv(1,2) = -((dx*Dxo + dy*Dyo )/r2 + 1);

    // Jacobian of the prediction model with respect to the landmark location
    //  Hp = [ dx/r   dy/r;
    //        -dy/r2  dx/r2];
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

    // update state vector and state covariance matrix
    updateMatrixOperationsDo();
}

void KalmanFilter::updateMatrixOperationsDo()
{
    // 2 x numStates to transfer full state covariance to innovation covariance S (2 x 2)
    _H.setZero();

    // Direct assignment to the blocks in matrix H
    _H.block<SLAM_ARRAY_SIZE::OBSERVATION_DIM, SLAM_ARRAY_SIZE::VEHICLE_STATE_DIM>(0, 0) = _Hv;
    _H.block<SLAM_ARRAY_SIZE::OBSERVATION_DIM, SLAM_ARRAY_SIZE::OBSERVATION_DIM>(0, _px) = _Hp;

    //Innovation update, kalman gain, state and covariance innovation
    _S  = _H * _X * _H.transpose() + _sigma_z;

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

    if (std::abs(_chi2) < SLAM_CONST::CHI_SQUARED)
    {
        // Kalman Gain (numStates x 2 )
        _K = _X * _H.transpose() * _S.inverse();

        // Update of the state vector and its covariance matrix
        _x  = _x + _K * _v;
        _X  = _X - _K * _S * _K.transpose();
    }
}

bool KalmanFilter::addState()
{
    // important, _z is set in isMapped(z), which must always be called before addState (modify code otherwise)

    // early return if the matching index was -1 (no observation on this step)
    if (_z(2) < 0.0)
    {
        return false;
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

    // z is expressed in range and bearing. transform it to a WRF (global X, Y)
    _obsWRF(0) = _x(0) + dxr + dxo;
    _obsWRF(1) = _x(1) + dyr + dyo;

    // Augment the mapped andmark list
    _mappedLandmarkList.push_back( _z(2));

    // Matrix operations
    addStateMatrixOperationDo();

    return true;
}

void KalmanFilter::addStateMatrixOperationDo()
{
    // Calculate mu (cross-covariance of new landmark with existing state), a N x OBSERVATION_DIM matrix
    _mu = Kalman::XVehicleAndStateBlockTallAndSkinny(_X) * _Tx.transpose();

    // Calculate Sigma (covariance of new landmark), a OBSERVATION_DIM x OBSERVATION_DIM matrix
    _sigma_new_landmark = _Tx * Kalman::XVehicleBlock(_X) * _Tx.transpose() + _Tz * _R * _Tz.transpose();

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
        _X.bottomRightCorner(_sigma_new_landmark.rows(), _sigma_new_landmark.cols()) = _sigma_new_landmark;

        // Set the top-right and bottom-left corners with mu
        _X.topRightCorner(_mu.rows(), _mu.cols()) = _mu;                   // This ensures correct dimensions
        _X.bottomLeftCorner(_mu.cols(), _mu.rows()) = _mu.transpose();

        // resize the state to innovation transfer matrix
        _H.resize(SLAM_ARRAY_SIZE::OBSERVATION_DIM, _x.rows());
        _H.setZero();

        // resize the kalman gain
        _K.resize(_x.rows(),SLAM_ARRAY_SIZE::OBSERVATION_DIM);
        _K.setZero();
    }
}

bool KalmanFilter::isMapped(const Kalman::ObservationWithTag& obsRB)
{
    // important, _z must be set here! Otherwise modify update() and addState() functions
    _z = obsRB;

    // And this is just for consistent logging
    _zpred.setZero();

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

void KalmanFilter::displayStateCovarianceMatrix(const std::optional<std::string>& message, std::optional<int> k) const
{
    if (message.has_value())
    {
        std::cout << message.value() << "(1:" << _X.rows() << ",1:" << _X.cols() << "," << k.value() << ") = [\n";
    }
    else
    {
        std::cout << "State covariance matrix _X:" << std::endl;
    }

    for (int i = 0; i < _X.rows(); ++i)
    {
        for (int j = 0; j < _X.cols(); ++j)
        {
            std::cout << std::setw(10) << std::setprecision(4) << _X(i, j) << " ";
        }
        std::cout << "\n";  // New line at the end of each row
    }

    if (message.has_value())
    {
        std::cout << "];\n\n";
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

void KalmanFilter::set_z(const Kalman::ObservationWithTag& z)
{
    _H.resize(SLAM_ARRAY_SIZE::OBSERVATION_DIM, _x.rows());
    _z = z;
}

DataVector KalmanFilter::eigenToStdVector(const Eigen::VectorXd& eigenVec) const
{
    return DataVector(eigenVec.data(), eigenVec.data() + eigenVec.size());
}

DataVector KalmanFilter::getVehicleStates() const
{
    return eigenToStdVector(_x.head(SLAM_ARRAY_SIZE::VEHICLE_STATE_DIM));
};

DataVector KalmanFilter::getVehicleStateStdev() const
{
    Kalman::VehicleState standard_deviations;
    for(int ii = 0; ii < SLAM_ARRAY_SIZE::VEHICLE_STATE_DIM; ++ii )
    {
        if (_X(ii,ii) >= 0)
        {
            standard_deviations(ii) = sqrt(_X(ii,ii));
        }
        else
        {
            standard_deviations(ii) = -1.0;
        }
    }
    return eigenToStdVector(standard_deviations.head(SLAM_ARRAY_SIZE::VEHICLE_STATE_DIM));
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


DataVector KalmanFilter::getInnovationCov() const
{
    // Extract the diagonal elements of the matrix _S
    Eigen::VectorXd diagonalElements = _S.diagonal();

    // if there was no match
    if (_z(2) < 0)
    {
        diagonalElements.setZero();
    }

    // Convert the Eigen vector to a std::vector<double>
    return eigenToStdVector(diagonalElements);
};

void KalmanFilter::testMatrixOperationsInPredict()
{

    // These are testing inputs given to the Matlab script
    _X.resize(8,8);
    _X <<  8.4685e-001,  4.1890e-001,  5.8177e-001,  3.9877e-001,  5.3903e-001,  2.3806e-001,  6.4224e-001,  2.9595e-001,
              6.9868e-001,  5.9315e-001,  6.8716e-001,  5.9029e-001,  3.1209e-001,  8.8598e-001,  5.8046e-001,  2.6830e-001,
              2.4588e-002,  1.8891e-001,  6.5284e-002,  1.1872e-001,  9.1743e-001,  7.9288e-001,  8.7550e-001,  4.5875e-001,
              5.5735e-001,  2.8237e-001,  9.7700e-001,  8.8549e-001,  6.9844e-001,  9.3385e-001,  7.3091e-001,  1.0670e-001,
              5.1053e-001,  9.5801e-001,  9.3576e-002,  1.8390e-001,  6.3294e-001,  2.4484e-001,  7.9044e-001,  1.6183e-002,
              6.4619e-001,  6.7986e-001,  2.6102e-001,  1.4340e-001,  1.7170e-001,  5.8678e-001,  9.6089e-001,  9.9757e-001,
              4.4299e-001,  4.7358e-001,  9.0126e-001,  4.2450e-001,  7.7275e-001,  3.4965e-001,  3.0428e-001,  2.6468e-001,
              4.8439e-001,  4.7782e-001,  3.6437e-001,  8.3193e-001,  5.4313e-001,  1.6681e-003,  7.2633e-001,  9.6793e-001;

    _sigma_u <<  1.9874e-001, 0,           0,
              0,           2.6074e-001, 0,
              0,           0,           6.0760e-001;

    _G << 2.7756e-001, 2.6892e-001, 0,
         3.4571e-001, 7.9669e-001, 0,
         3.5601e-001, 2.8520e-001, 0,
         0,           0,           1.0000e+000;

    _F << 1.0000e+000, 0,           9.5570e-002, 1.9626e-001,
         0,           1.0000e+000, 8.1798e-001, 7.2559e-002,
         0,           0,           1.0000e+000, 8.5142e-001,
         0,           0,           0,           1.0000e+000;

    // This is the expected output from the Matlab script
    Kalman::StateCovariance X_expected_result(8, 8);   
    X_expected_result <<
        1.181870292017600e+00, 1.247496222120209e+00, 1.316537372506980e+00, 5.839023378000000e-01, 7.637846195000000e-01, 4.971129425999999e-01, 8.693599316000000e-01, 3.607336795000000e-01,
        1.059235253214821e+00, 1.675701001167513e+00, 1.535126183964478e+00, 7.516508545099999e-01, 1.113207499360000e+00, 1.602299204550000e+00, 1.349635588690000e+00, 6.512903703000000e-01,
        7.957659466308120e-01, 1.310175411626126e+00, 1.686505074740310e+00, 8.726438958000000e-01, 1.512095784800000e+00, 1.587978567000000e+00, 1.497811392200000e+00, 5.495965140000000e-01,
        8.245081574000001e-01, 1.145786728910000e+00, 1.730923895800000e+00, 1.493090000000000e+00, 6.984399999999999e-01, 9.338500000000000e-01, 7.309099999999999e-01, 1.067000000000000e-01,
        5.555652723200000e-01, 1.047896896580000e+00, 2.501521380000000e-01, 1.839000000000000e-01, 6.329399999999999e-01, 2.448400000000000e-01, 7.904400000000000e-01, 1.618300000000000e-02,
        6.992793654000000e-01, 9.037741001999999e-01, 3.831136280000000e-01, 1.434000000000000e-01, 1.717000000000000e-01, 5.867800000000000e-01, 9.608900000000000e-01, 9.975700000000000e-01,
        6.124357882000000e-01, 1.241593950300000e+00, 1.262687790000000e+00, 4.245000000000000e-01, 7.727500000000000e-01, 3.496500000000000e-01, 3.042800000000000e-01, 2.646800000000000e-01,
        6.824874227000000e-01, 8.362313814700001e-01, 1.072691840600000e+00, 8.319299999999999e-01, 5.431300000000000e-01, 1.668100000000000e-03, 7.263300000000000e-01, 9.679300000000000e-01;

    // DO THE TEST
    predictMatrixOperationsDo();

    // Compare Eigen calcs with Matlab expected outputs. This will throw an error if failure and display the matrix
    testEigenMaxtrixEquality(_X, X_expected_result,"_X","predict()");

    // reset state of matrix
    _X.resize(SLAM_ARRAY_SIZE::VEHICLE_STATE_DIM,SLAM_ARRAY_SIZE::VEHICLE_STATE_DIM);
    _sigma_u.setZero();
    _G.setZero();
    _F.setZero();
}


void KalmanFilter::testMatrixOperationsInUpdate(const Kalman::State& x_init)
{
    // 2nd landmark first state(index #1 as 0-based in C) is index 6 in the 0-based covariance/state matrix
    _px = 6;

    // These are testing inputs given to the Matlab script
    _X.resize(10,10);
    _X <<   0.4218, 0.7431, 0.6948, 0.4456, 0.9597, 0.5472, 0.1966, 0.7572, 0.5688, 0.2630,
            0.9157, 0.3922, 0.3171, 0.6463, 0.3404, 0.1386, 0.2511, 0.7537, 0.4694, 0.6541,
            0.7922, 0.6555, 0.9502, 0.7094, 0.5853, 0.1493, 0.6160, 0.3804, 0.0119, 0.6892,
            0.9595, 0.1712, 0.0344, 0.7547, 0.2238, 0.2575, 0.4733, 0.5678, 0.3371, 0.7482,
            0.6557, 0.7060, 0.4387, 0.2760, 0.7513, 0.8407, 0.3517, 0.0759, 0.1622, 0.4505,
            0.0357, 0.0318, 0.3816, 0.6797, 0.2551, 0.2543, 0.8308, 0.0540, 0.7943, 0.0838,
            0.8491, 0.2769, 0.7655, 0.6551, 0.5060, 0.8143, 0.5853, 0.5308, 0.3112, 0.2290,
            0.9340, 0.0462, 0.7952, 0.1626, 0.6991, 0.2435, 0.5497, 0.7792, 0.5285, 0.9133,
            0.6787, 0.0971, 0.1869, 0.1190, 0.8909, 0.9293, 0.9172, 0.9340, 0.1656, 0.1524,
            0.7577, 0.8235, 0.4898, 0.4984, 0.9593, 0.3500, 0.2858, 0.1299, 0.6020, 0.8258;

    _x.resize(10,1);
    _x << 0.5383, 0.9961, 0.0782, 0.4427, 0.1067, 0.9619, 0.0046, 0.7749, 0.8173, 0.8687;

    _Hv <<  0.8147, 0.1270, 0.6324, 0.2785,
            0.9058, 0.9134, 0.0975, 0.5469;

    _Hp << 0.9575, 0.1576,
           0.9649, 0.9706;

    _sigma_z << 0.8003, 0,
             0, 0.1419;

    _v << 0.9572, 0.4854;

    _H.resize(SLAM_ARRAY_SIZE::OBSERVATION_DIM, _x.rows());
    _H.setZero();

    // This is the expected output from the Matlab script
    Kalman::ObservationCovariance S_expected_output(2, 2);
    S_expected_output << 6.138489529834000, 7.418674125547000,
                         7.536041833706000, 10.87500939420500;

    Kalman::KalmanGain K_expected_output(10, 2);
    K_expected_output <<   -0.2832763064911634, 0.4044527895019859,
                           -0.2292018093205031, 0.3904600215423526,
                            0.2645864689518467, 0.07334592917612826,
                           -0.1195793217007177, 0.3068045607049779,
                            0.04809354592100656, 0.1368959401601376,
                            0.3508481485382010, -0.1175594066371730,
                            0.2816321355685667, 0.04097091568236766,
                            0.3424369616881071, -0.01830407221648578,
                           -0.04477229706150943, 0.2676282020779155,
                           -0.03280729311133213, 0.2210642684883879;

    Kalman::State x_expected_output(10);
    x_expected_output << 0.4634693034509224,
                         0.9662373225750724,
                         0.3670642821028003,
                         0.4771616070342692,
                         0.2191844315093183,
                         1.240668511799082,
                         0.2940655626384533,
                         1.093795863073974,
                         0.9043506865413434,
                         0.9446014549580964;

    Kalman::StateCovariance X_expected_output(10, 10);
    X_expected_output << -0.1363522722395347,  0.1462995363641411,  0.1799661827454017, -0.1025578971976925,  0.5823061194716700,  0.3579315380006538, -0.2661860842809860,  0.3509635816976863,  0.01269730122137147, -0.2018135138400922,
                          0.3210012119332599, -0.2499100235459500, -0.2759212284900952,  0.04883396142419128, -0.08197204255019114, -0.1008594645130841, -0.2856730530637980,  0.2744640472822437, -0.1431990522844767,  0.1416741656903949,
                          0.2923645903574207,  0.07657715174080249,  0.1717491200761576,  0.1227724834833126,  0.1027014255626697, -0.2899361627566171, -0.1101857565757117, -0.3145220392487064, -0.6294278119109958,  0.1503661780880712,
                          0.4158661904686976, -0.4234927316607262, -0.5627843888042331,  0.1919211074194485, -0.1874048036570163, -0.008209933416758741, -0.07148460810014407,  0.07224911181910099, -0.2477706503159257,  0.2585007905189186,
                          0.2851405520428101,  0.2895145822866475, -0.04773571771811080, -0.1315517052323068,  0.4348396175662216,  0.5921282688648030, -0.09760397116133723, -0.3446903270215466, -0.2723099552650142,  0.08604934525356339,
                         -0.1408223620071210, -0.1947434596146457, -0.05096522731815656,  0.4249738223781310,  0.01282898243563446, -0.03479404733660912,  0.4195016584851369, -0.3558814380062924,  0.4966215175103754, -0.1673792537570555,
                          0.4010450879943451, -0.2458644309983837,  0.04046635075755634,  0.1210316854618806,  0.06098979256772930,  0.3974286786013938, -0.09239782669652641, -0.1205350333072718, -0.2759837006117654, -0.2645283126010463,
                          0.5434103960238569, -0.4178353536865847,  0.1037942229909808, -0.3208769864799232,  0.2841149830761404, -0.1703525434477658, -0.09938333724964865,  0.1494743260786329, -0.01017947071945602,  0.4601434561374372,
                          0.1293699400523218, -0.5103514130446022, -0.4631241331294423, -0.4637267584807079,  0.4541600189704965,  0.6212188120872291,  0.3209546832214385,  0.3846604577752226, -0.4464335957521761, -0.3604207398880669,
                          0.2986789661457021,  0.3155175186215554, -0.05618115098341070,  0.01062201635059834,  0.5930743992969730,  0.09017037252378479, -0.2151932447075856, -0.3320708668878799,  0.08931040658105660,  0.3961971090759432;

    // Calculate Matrix operations
    updateMatrixOperationsDo();

    // Compare Eigen calcs with Matlab expected outputs. This will throw an error if failure and display the matrix
    testEigenMaxtrixEquality(_X, X_expected_output,"_X", "update()");
    testEigenMaxtrixEquality(_x, x_expected_output,"_x", "update()");
    testEigenMaxtrixEquality(_K, K_expected_output,"_K", "update()");
    testEigenMaxtrixEquality(_S, S_expected_output,"_S", "update()");

    // reset state of matrix
    _X.resize(SLAM_ARRAY_SIZE::VEHICLE_STATE_DIM,SLAM_ARRAY_SIZE::VEHICLE_STATE_DIM);
    _X.setZero();
    _x.resize(SLAM_ARRAY_SIZE::VEHICLE_STATE_DIM);
    _x = x_init;
    _Hv.setZero();
    _Hp.setZero();
    _H.resize(SLAM_ARRAY_SIZE::OBSERVATION_DIM, SLAM_ARRAY_SIZE::VEHICLE_STATE_DIM);
    _H.setZero();
    _K.resize(SLAM_ARRAY_SIZE::VEHICLE_STATE_DIM, SLAM_ARRAY_SIZE::OBSERVATION_DIM);
    _K.setZero();
    _S.setZero();
    _sigma_z.setZero();
    _v.setZero();
}


void KalmanFilter::testMatrixOperationsInAddState(const Kalman::State& x_init)
{
     // These are testing inputs given to the Matlab script
    _X.resize(6,6);
    _X <<   0.54986, 0.40181, 0.41727, 0.33772, 0.24169, 0.57521,
            0.14495, 0.075967, 0.049654, 0.90005, 0.40391, 0.05978,
            0.85303, 0.23992, 0.90272, 0.36925, 0.096455, 0.23478,
            0.62206, 0.12332, 0.94479, 0.1112, 0.13197, 0.35316,
            0.35095, 0.18391, 0.49086, 0.78025, 0.94205, 0.82119,
            0.51325, 0.23995, 0.48925, 0.38974, 0.95613, 0.015403;

    // This is needed as length of that vector used in addStateMatrixOperationDo
    _x.resize(6,1);

    _R << 0.64912, 0,
         0, 0.73172;

    _Tx << 1, 0, 0.43141, 0,
          0, 1, 0.91065, 0;

    _Tz << 0.14554, 0.86929,
          0.13607, 0.5797;


    // Initialize expected output matrix (8x8)
    Kalman::StateCovariance new_expected(8, 8);
    new_expected << 0.54986, 0.40181, 0.41727, 0.33772, 0.24169, 0.57521, 0.7298744507, 0.7817969254999999,
                    0.14495, 0.075967, 0.049654, 0.90005, 0.40391, 0.05978, 0.16637123214, 0.1211844151,
                    0.85303, 0.23992, 0.90272, 0.36925, 0.096455, 0.23478, 1.2424724352, 1.061981968,
                    0.62206, 0.12332, 0.94479, 0.1112, 0.13197, 0.35316, 1.0296518539, 0.9836930134999999,
                    0.35095, 0.18391, 0.49086, 0.78025, 0.94205, 0.82119, 0.5627119126, 0.630911659,
                    0.51325, 0.23995, 0.48925, 0.38974, 0.95613, 0.015403, 0.7243173425, 0.6854855125,
                    0.7298744507, 0.16637123214, 1.2424724352, 1.0296518539, 0.5627119126, 0.7243173425, 1.832574343417076, 1.621535263832776,
                    0.7817969254999999, 0.1211844151, 1.061981968, 0.9836930134999999, 0.630911659, 0.6854855125, 1.679417452772776, 1.346192815499488;

    // Calculate Matrix operations
    addStateMatrixOperationDo();

    // Compare Eigen calcs with Matlab expected outputs. This will throw an error if failure and display the matrix
    testEigenMaxtrixEquality(_X, new_expected,"_X", "addState()");

    // reset state of matrix
    _X.resize(SLAM_ARRAY_SIZE::VEHICLE_STATE_DIM,SLAM_ARRAY_SIZE::VEHICLE_STATE_DIM);
    _x.resize(SLAM_ARRAY_SIZE::VEHICLE_STATE_DIM);
    _x = x_init;
    _X.setZero();
    _R.setZero();
    _Tx.setZero();
    _Tz.setZero();
}
