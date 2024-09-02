// KalmanFilter.cpp
//
// NOTE: the Matlab cpp math library is used herein, the indexing syntax for mwArray object
// is the same as in Matlab (i.e: starts from 1 and not 0)

#include "KalmanFilter.h"


CKalmanFilter::CKalmanFilter()
{
//	m_vStDevBeaconLoc =
    m_vStDevVehicleLocXY = 0;
    m_vStDevVehicleLocTheta = 0;

    m_vStDevBeaconLoc = 0;
    m_QEnabled = false;


    m_wStDevRange = 0;
    m_wStDevTheta = 0;
    m_REnabled = 0;

    m_var_q = 0;
    m_var_w = 0;
    m_var_s = 0;
    m_var_g = 0;
    m_var_r = 0;
    m_var_range = 0;
    m_var_bearing = 0;
}


// Function to generate a one-step vehicle prediction from previous estimate and control input.
void CKalmanFilter::PredictState(Kalman::Input& u)
{
    // x(k+1|k) is based on the process model and knowledge of the control input u(k)
    // For the first step, there is no uncertainty
    // and state prediction should be the initial condition
    // x(k) = [X(k) Y(k) Theta(k) R(k) Xb1 Yb1 ... XbN YbN]'
    // u(k) = [wheel_angular_velocity(k), wheel_steer_angle(k), wheel_radius_noise]'

    // angle of front wheel in the global reference frame
    double phi = _x(2) + u(1);

    // The fourth state is the wheel radius (to accomodate for elastic deformation)
    double R = _x(3,0) + u(2);

    // Source error transfer matrix
    _G(0,0) =  _dt*cos( phi );
    _G(0,1) = -_dt*sin( phi );
    _G(1,0) =  _dt*sin( phi );
    _G(1,1) =  _dt*cos( phi );
    _G(2,0) =  _dt*sin( u(2) ) / SlamPhysicalConstants::WHEEL_BASE;
    _G(2,1) =  _dt*cos( u(2) ) / SlamPhysicalConstants::WHEEL_BASE;
    _G(3,2) =  _dt;

    // state transition matrix
    _F(0,0) =  1.0;
    _F(2,0) = -_dt*R*u(0)*sin( phi );
    _F(3,0) =  _dt*u(0)*cos( phi );
    _F(1,1) =  1.0;
    _F(1,2) =  _dt*R*u(0)*cos( phi );
    _F(2,3) =  _dt*u(0)*sin( phi );
    _F(2,2) =  1.0;
    _F(2,3) =  _dt*u(1)*sin( u(1) ) / SlamPhysicalConstants::WHEEL_BASE;
    _F(3,3) =  1.0;

    // source error covariance
    _sigma_u(0,0) = (m_var_q*R*u(0))*(m_var_q*R*u(0)) + m_var_w*R*R;
    _sigma_u(1,1) = (m_var_s*R*u(0)*u(0))*(m_var_s*R*u(0)*u(1)) + (m_var_g*R*u(0))*(m_var_g*R*u(0));
    _sigma_u(2,2) = m_var_r;

    // First state prediction
    _x(0) = _x(0) + _dt*R*u(0)*cos( phi );
    _x(1) = _x(1) + _dt*R*u(0)*sin( phi );
    _x(2) = _x(2) + _dt*R*u(0)*sin(u(1)) / SlamPhysicalConstants::WHEEL_BASE;
    _x(3) = R;

    // Prediction cross covariance for the vehicle - writing/reading to/from X by reference
    Kalman::XVehicleBlock(_X) = _F * Kalman::XVehicleBlock(_X) * _F.transpose() + _G * _sigma_u * _G.transpose();

    // Prediction cross covariance between the vehicle and landmarks - writing/reading to/from X by reference
    Kalman::VehicleToLandmarkCrossCovariance(_X) = Kalman::VehicleToLandmarkCrossCovariance(_X) * _F.transpose();

    // Prediction cross covariance between the landmarks and the vehicle - writing/reading to/from X by reference
    Kalman::LandmarkToVehicleCrossCovariance(_X) = _F.transpose() * Kalman::LandmarkToVehicleCrossCovariance(_X);

    // The prediction covariance of the landmarks does not change in this step!

    // For all subsequent calls, add noise by setting the sampling period to its actuall value
    _dt = SlamConstants::DT;
}

void CKalmanFilter::UpdateEKF(Kalman::ObservationWithTag& obsRB)
{
    // Subscripts for accessing observed landmark state and Covariance.
    // landmark tags (in obsRB(2)) start at 0 for first landmark
    int px = SlamArraySize::VEHICLE_STATE_DIM + SlamArraySize::OBSERVATION_DIM*obsRB(2);

    // Predict the observation based on the vehicle's position
    // and the landmark position
    double dx = _x(px) - ( _x(0) + SlamPhysicalConstants::XOFFSET*cos( _x(2) ) - SlamPhysicalConstants::YOFFSET*sin( _x(2) ) );
    double dy = _x(px+1) - ( _x(1) + SlamPhysicalConstants::XOFFSET*sin( _x(2) ) + SlamPhysicalConstants::YOFFSET*cos( _x(2) ) ) ;

    // Prediction model
    double r2 = dx*dx + dy*dy;
    double r  = sqrt( r2 );

    // get orientation between 0 and 360deg
    // phi=pi2pi(x(3)-(2.0*pi*fix(x(3)/(2.0*pi))));
    double phi = _x(2);

    // Jacobian of the prediction model with respect to the state
    double Dxo = SlamPhysicalConstants::XOFFSET*cos(_x(2)) - SlamPhysicalConstants::YOFFSET*sin(_x(2));
    double Dyo = SlamPhysicalConstants::XOFFSET*sin(_x(2)) + SlamPhysicalConstants::YOFFSET*cos(_x(2));

    _Hv(0,0) = -dx/r;
    _Hv(0,1) = -dy/r;
    _Hv(0,2) = (dx*Dyo - dy*Dxo)/r;
    _Hv(0,1) = dy/r2;
    _Hv(1,1) = -dx/r2;
    _Hv(1,2) = -((dx*Dxo + dy*Dyo )/r2 + 1);

    // Jacobian of the prediction model with respect to the landmark location
    _Hp(0,0) = dx/r;
    _Hp(0,1) = dy/r;
    _Hp(1,0) = -dy/r2;
    _Hp(1,1) = dx/r2;

    // observation model
    _zpred(0) = r;
    _zpred(1) = a_sub( atan2(dy,dx), phi );  //normalisation

    // observation z and its covariance matrix exprimed in range and bearing
    _sigma_z(0,0) = m_wStDevRange;
    _sigma_z(1,1) = m_wStDevTheta;

    _v(0) = obsRB(0) - _zpred(0);
    _v(1) = a_sub( obsRB(1) - _zpred(1), 0 );

    // 2 x numStates to transfer full state covariance to innovation covariance (2 x 2)
    Kalman::FullStateToInovationTransition H = Kalman::FullStateToInovationTransition::Zero(SlamArraySize::OBSERVATION_DIM, _x.rows());

    // Assign Hpi to Hi(i2, landIndex)
    Kalman::VehicleStateToInnovationCovariance(H) = _Hv;

    Kalman::LandmarkStateToInnovationCovariance(H, px) = _Hp;

    //Innovation update, kalman gain, state and covariance innovation
    _S  = H*_X*H.transpose() + _sigma_z;

    // Compute chi2
    double chi2 = _v.transpose() * _S.inverse() * _v;

    if ( chi2 < 7.5)
    {
        // Kalman Gain (numStates x 2 )
        Kalman::KalmanGain K = _X*H.transpose()*_S.inverse();

        // Update of the state vector and its covariance matrix
        _x  = _x + K*_v;
        _X  = _X - K*_S*K.transpose();
    }
    else
    {
        std::cout << "Innovation gate failed" << endl;
    }
}

void CKalmanFilter::CorruptControls(Eigen::VectorXd utrue, Eigen::VectorXd& u)
{
    // NOTE: the Matlab cpp math library is used herein, the indexing syntax for mwArray object
    // is the same as in Matlab (i.e: starts from 1 and not 0)

//	int32 size[2];
//	u.Size(size);

//	::cout << size[0] << " " << size[1] << endl;

    double a = (1+sqrt(m_var_q)*rand())*utrue(0) + sqrt(m_var_w)*rand(); // wheel velocity
    double b = (1+sqrt(m_var_s)*rand())*utrue(1) + sqrt(m_var_g)*rand(); // steer angle
    double c = utrue(2); // time

    // Set up the random number generator
    std::random_device rd;  // Seed generator
    std::mt19937 gen(rd()); // Mersenne Twister RNG
    std::normal_distribution<> normal_dist(0.0, 1.0); // Mean = 0, StdDev = 1

    // Calculate the additive wheel noise
    double d = sqrt(m_var_r) * normal_dist(gen); // not sure about that  * sqrt(DT);

    u(0) = a;
    u(1) = b;
    u(2) = c;
    u(3) = d;

//	::cout << u << endl;
}

void CKalmanFilter::AddState(Eigen::VectorXd& z)
{
    double phi = _x(2) + z(1);
    double dxo = z(0)*cos( phi );
    double dyo = z(0)*sin( phi );
    double dxr = SlamPhysicalConstants::XOFFSET*cos( _x(2) ) - SlamPhysicalConstants::YOFFSET*sin( _x(2) );
    double dyr = SlamPhysicalConstants::XOFFSET*sin( _x(2) ) + SlamPhysicalConstants::YOFFSET*cos( _x(2) );

    // z is expressed in range and bearing transform it to a WRF location
    _obsWRF(0) = _x(0) + dxr + dxo;
    _obsWRF(1) = _x(1) + dyr + dyo;

    // Transformation matrix (2 x 4)
    _Tx(0,0) = 1.0;
    _Tx(0,2) = -dyr + dyo;
    _Tx(1,1) = 1.0;
    _Tx(1,2) = dxr + dxo;

    // Transformation matrix (2 x 2)
    _Tz(0,0) = cos( phi );
    _Tz(0,1) = -dyo;
    _Tz(1,0) = sin( phi );
    _Tz(1,1) = dxo;

    // Augment
    _R(0,0) = sqrt( m_var_range );
    _R(1,1) = sqrt( m_var_bearing );

    // Calculate mu (cross-covariance of new landmark with existing state), a N x OBSERVATION_DIM matrix
    Eigen::MatrixXd mu = Kalman::XVehicleAndStateBlockTallAndSkinny(_X) * _Tx.transpose();

    // Calculate Sigma (covariance of new landmark), a OBSERVATION_DIM x OBSERVATION_DIM matrix
    Eigen::Matrix2d Sigma = _Tx * Kalman::XVehicleBlock(_X) * _Tx.transpose() + _Tz * _R * _Tz.transpose();

    // Resize the state vector x and state covariance X
    int newStateStart = _x.rows();
    int requiredRows = newStateStart + _obsWRF.cols();
    if (newStateStart < requiredRows) {
        _x.conservativeResize(requiredRows, _x.cols());
        _x.block(newStateStart, 0, _obsWRF.cols(), _x.cols()) = _obsWRF;

        // Update X with mu and Sigma
        _X.conservativeResize(requiredRows, requiredRows);

        // Set the bottom-right corner with Sigma
        _X.bottomRightCorner(Sigma.rows(), Sigma.cols()) = Sigma;

        // Set the top-right and bottom-left corners with mu
        _X.topRightCorner(mu.rows(), mu.cols()) = mu;                   // This ensures correct dimensions
        _X.bottomLeftCorner(mu.cols(), mu.rows()) = mu.transpose();
    }

    // Augment the mapped landmark list
    m_mappedLandmarkList.push_back( z[2]);
}

bool CKalmanFilter::isMapped(Eigen::VectorXd& obsRB)
{
    int numMapped = m_mappedLandmarkList.size();
    bool result = false;

    for( int i = 0; i < numMapped; i++ )
        if( m_mappedLandmarkList[i] == obsRB(2) )
            result = true;

    return result;
}
