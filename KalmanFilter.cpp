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
void CKalmanFilter::PredictState(Eigen::VectorXd& x, Eigen::MatrixXd& X, Eigen::VectorXd& u)
{
    // x(k+1|k) is based on the process model and knowledge of the control input u(k)
    // x(k+1) and u(k) have the same index in the storing array. A single subtility:
    // for i=0, the true or predicted state must NOT be based on u(:,0) but on the initial conditions !!!
    // We want the prediction x,y,phi,r array to begin with the
    // initial state, which is achievied by setting dt=0

    // Source error covariance (3x3 matrix of zeros)
    Eigen::Matrix<double, UTRUE_DIM, UTRUE_DIM> sigma = Eigen::Matrix<double, UTRUE_DIM, UTRUE_DIM>::Zero();  // 3x3 zero matrix

    // Source error transfer matrix (4x3 matrix of zeros)
    Eigen::Matrix<double, XTRUE_DIM, UTRUE_DIM> G = Eigen::Matrix<double, XTRUE_DIM, UTRUE_DIM>::Zero();  // 4x3 zero matrix

    // State transition matrix (4x4 matrix of zeros)
    Eigen::Matrix<double, XTRUE_DIM, XTRUE_DIM> F = Eigen::Matrix<double, XTRUE_DIM, XTRUE_DIM>::Zero();  // 4x4 zero matrix

    double dt, R, phi;

    // For the first step, there is no uncertainty
    // Wheras the control vector returned from getControl has three states (vel, steer, time stamps)
    // corruptcontrols() adds a fourth one: wheel radius noise
    dt = (u(2) == 0) ? 0 : DT;

    // The fourth's state is the wheel radius (to accomodate for elastic deformation
//	::cout << u << endl;
//	::cout << u(4,1) << endl;
    R = double( x(3,0) ) + double( u(3,0) );
    phi = x(2) + u(1);


    // Source error transfer matrix
    G(0,0) =  dt*cos( phi );
    G(0,1) = -dt*sin( phi );
    G(1,0) =  dt*sin( phi );
    G(1,1) =  dt*cos( phi );
    G(2,0) =  dt*sin( u(2) )/WHEEL_BASE;
    G(2,1) =  dt*cos( u(2) )/WHEEL_BASE;
    G(3,2) =  dt;

//	::cout << G << endl;

    // state transition matrix
    F(0,0) =  1.0;
    F(2,0) = -dt*R*u(0)*sin( phi );
    F(3,0) =  dt*u(0)*cos( phi );
    F(1,1) =  1.0;
    F(1,2) =  dt*R*u(0)*cos( phi );
    F(2,3) =  dt*u(0)*sin( phi );
    F(2,2) =  1.0;
    F(2,3) =  dt*u(1)*sin( u(1) )/WHEEL_BASE;
    F(3,3) =  1.0;

//	::cout << F << endl;

    // source error covariance
    sigma(0,0) = (m_var_q*R*u(0))*(m_var_q*R*u(0)) + m_var_w*R*R;
    sigma(1,1) = (m_var_s*R*u(0)*u(0))*(m_var_s*R*u(0)*u(1)) + (m_var_g*R*u(0))*(m_var_g*R*u(0));
    sigma(2,2) = m_var_r;

//	::cout << sigma << endl;

    // First state prediction
    x(0) = x(0) + dt*R*u(0)*cos( phi );
    x(1) = x(0) + dt*R*u(0)*sin( phi );
    x(2) = x(0) + dt*R*u(0)*sin(u(1))/WHEEL_BASE;
    x(3) = R;

//	::cout << x << endl;

    // Define the vehicle index block (assuming it starts at (0,0) for simplicity)
    Eigen::Block<Eigen::MatrixXd> X_vehicle = X.block(0, 0, XTRUE_DIM, XTRUE_DIM);

    // Prediction cross covariance for the vehicle
    Eigen::Matrix<double, XTRUE_DIM, XTRUE_DIM> A = F * X_vehicle * F.transpose();
    Eigen::Matrix<double, XTRUE_DIM, XTRUE_DIM> B = G * sigma * G.transpose();
    X_vehicle = A + B;

    // Prediction cross covariance between the vehicles and landmarks
    // The prediction covariance of the landmarks does not change in this step

    // Define indices for blocks
    int landmarkStartRow = X_VEHICLE_DIM;
    int vehicleEndRow = X_VEHICLE_DIM;

    // Find out the size of the state vector, the control vector has always a fixed size of 4
    int numStates = x.rows();  // Get the number of rows in the matrix

    // Prediction cross covariance between the vehicle and landmarks
    // For a more efficient implementation pick out the
    // cross-covariance matrixes between the vehicle's state
    // and the rest of the state vector (the beacons)

    // Equivalent to X(landmarkIndex, vehicleIndex) = X(landmarkIndex, vehicleIndex) * transpose(F);
    X.block(landmarkStartRow, 0, numStates - X_VEHICLE_DIM, X_VEHICLE_DIM) =
        X.block(landmarkStartRow, 0, numStates - X_VEHICLE_DIM, X_VEHICLE_DIM) * F.transpose();

    // Equivalent to X(vehicleIndex, landmarkIndex) = transpose(F) * X(vehicleIndex, landmarkIndex);
    X.block(0, landmarkStartRow, X_VEHICLE_DIM, numStates - X_VEHICLE_DIM) =
        F.transpose() * X.block(0, landmarkStartRow, X_VEHICLE_DIM, numStates - X_VEHICLE_DIM);


}

void CKalmanFilter::UpdateEKF( Eigen::VectorXd& x, Eigen::MatrixXd& X, Eigen::VectorXd& obsRB)
{
    // NOTE: the Matlab cpp math library is used herein, the indexing syntax for mwArray object
    // is the same as in Matlab (i.e: starts from 1 and not 0)

    // Find out the size of the state vector, the control vector has always a fixed size of 4
    int numStates = x.rows();  // Get the number of rows in the matrix

    double dx, dy, r, r2, Dxo, Dyo;

    // 2x4 matrix of zeros
    Eigen::Matrix<double, 2, 4> Hv = Eigen::Matrix<double, 2, 4>::Zero();

    // 2x2 matrix of zeros
    Eigen::Matrix2d Hpi = Eigen::Matrix2d::Zero();  // or Eigen::Matrix<double, 2, 2>::Zero();

    // 2x(numStates) matrix of zeros
    Eigen::MatrixXd Hi = Eigen::Matrix<double, 2, Eigen::Dynamic>::Zero(2, numStates);

    // 2x2 matrix of zeros (Kalman Gain)
    Eigen::Matrix2d Wi = Eigen::Matrix2d::Zero();

    // 2x1 vector of zeros (predicted measurement)
    Eigen::Vector2d zpred = Eigen::Vector2d::Zero();

    // 2x1 vector of zeros (measurement)
    Eigen::VectorXd z = Eigen::VectorXd::Zero(2);  // 2x1 vector of zeros

    // 2x1 vector of zeros (measurement residual)
    Eigen::Vector2d v = Eigen::Vector2d::Zero();

    // 2x2 matrix of zeros (measurement covariance)
    Eigen::Matrix2d sigma = Eigen::Matrix2d::Zero();

    // 2x2 matrix of zeros (innovation covariance)
    Eigen::Matrix2d S = Eigen::Matrix2d::Zero();


    // Subscripts for state and covariance information. Extraction of the observed
    // landmark. obsRB[2] is the index (or tag) of the observed landmark (0...N)
    int px = 4 + 2*(obsRB(1)) + 1;
    int py = px + 1;



    // Predict the observation based on the vehicle's position
    // and the landmark position
    dx = double( x(px) - ( x(0) + XOFFSET*cos( x(2) ) - YOFFSET*sin( x(2) ) ) ); // The term in parentesis is the radar x-location
    dy = double( x(py) - ( x(1) + XOFFSET*sin( x(2) ) + YOFFSET*cos( x(2) ) ) );

    // Prediction model
    r2 = dx*dx + dy*dy;
    r  = sqrt( r2 );

    // get orientation between 0 and 360deg
    // phi=pi2pi(x(3)-(2.0*pi*fix(x(3)/(2.0*pi))));
    double phi = x(2);

    // Jacobian of the prediction model with respect to the state
    Dxo = XOFFSET*cos(x(2)) - YOFFSET*sin(x(2));
    Dyo = XOFFSET*sin(x(2)) + YOFFSET*cos(x(2));

    Hv(0,0) = -dx/r;
    Hv(0,1) = -dy/r;
    Hv(0,2) = (dx*Dyo - dy*Dxo)/r;
    Hv(0,1) = dy/r2;
    Hv(1,1) = -dx/r2;
    Hv(1,2) = -((dx*Dxo + dy*Dyo )/r2 + 1);


    // Jacobian of the perdiction model with respect to the landmark location
    Hpi(0,0) = dx/r;
    Hpi(0,1) = dy/r;
    Hpi(1,0) = -dy/r2;
    Hpi(1,1) = dx/r2;

    // observation model
    zpred(0) = r;
    zpred(1) = a_sub( atan2(dy,dx), phi );  //normalisation

    z(0) = obsRB(0);
    z(1) = obsRB(1);

    // observation z and its covariance matrix exprimed in range and bearing
    sigma(0,0) = m_wStDevRange;
    sigma(1,1) = m_wStDevTheta;
    v(0) = z(0) - zpred(0);
    v(1) = a_sub( z(1) - zpred(1), 0 );

    // Hpi has to multiply the matched landmark covariance matrix
    int i2_start = 0;  // Rows 1 to 2 in MATLAB -> 0-based index in Eigen
    int i4_start = 0;  // Columns 1 to 4 in MATLAB -> 0-based index in Eigen
    int i2_size = 2;   // 2 rows
    int i4_size = 4;   // 4 columns

    // Assign Hv to Hi(i2, i4)
    Hi.block(i2_start, i4_start, i2_size, i4_size) = Hv;

    int landIndex_start = px - 1;  // Convert MATLAB 1-based to 0-based index
    int landIndex_size = py - px + 1;  // Calculate the number of columns in the range

    // Assign Hpi to Hi(i2, landIndex)
    Hi.block(i2_start, landIndex_start, i2_size, landIndex_size) = Hpi;

    //Innovation update, kalman gain, state and covariance innovation
    S  = Hi*X*Hi.transpose() + sigma;

    // Compute chi2
    double chi2 = v.transpose() * S.inverse() * v;

    if ( chi2 < 7.5)
    {
        // Kalman Gain Wi
        Wi = X*Hi.transpose()*S.inverse();

        // Update of the state vector and its covariance matrix
        x  = x + Wi*v;
        X  = X - Wi*S*Wi.transpose();
    }
    else
        std::cout << "Innovation gate failed" << endl;

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

void CKalmanFilter::AddState(Eigen::VectorXd& x, Eigen::MatrixXd& X, Eigen::VectorXd& z)
{
    // NOTE: the Matlab cpp math library is used herein, the indexing syntax for mwArray object
    // is the same as in Matlab (i.e: starts from 1 and not 0)

    // find out the size of the state vector
    int numStates = x.rows();  // Get the number of rows in the matrix

    // Initialize Tx as a 2x4 matrix of zeros
    Eigen::Matrix<double, BEACON_DIM, X_VEHICLE_DIM> Tx = Eigen::Matrix<double, BEACON_DIM, X_VEHICLE_DIM>::Zero();

    // Initialize Tz as a 2x2 matrix of zeros
    Eigen::Matrix<double, BEACON_DIM, BEACON_DIM> Tz = Eigen::Matrix<double, BEACON_DIM, BEACON_DIM>::Zero();

    // Initialize R (Noise covariance matrix) as a 2x2 matrix of zeros
    Eigen::Matrix<double, BEACON_DIM, BEACON_DIM> R = Eigen::Matrix<double, BEACON_DIM, BEACON_DIM>::Zero();

    // Initialize obsWRF (Observation matrix) as a 2x1 matrix of zeros
    Eigen::Matrix<double, BEACON_DIM, 1> obsWRF = Eigen::Matrix<double, BEACON_DIM, 1>::Zero();

    // Some variable
    double dxo, dyo, dxr, dyr, phi;

    phi = x(2) + z(1);
    dxo = z(0)*cos( phi );
    dyo = z(0)*sin( phi );
    dxr = XOFFSET*cos( x(2) ) - YOFFSET*sin( x(2) );
    dyr = XOFFSET*sin( x(2) ) + YOFFSET*cos( x(2) );

    // z is expressed in range and bearing transform it to a WRF location
    obsWRF(0) = x(0) + dxr + dxo;
    obsWRF(1) = x(1) + dyr + dyo;

    int vehicleSize = X_VEHICLE_DIM;
    int newStateStart = numStates;
    int newStateSize = 2;

    // Check if x needs resizing
    int requiredRows = newStateStart + newStateSize;
    if (x.rows() < requiredRows) {
        x.conservativeResize(requiredRows, x.cols());  // Resize only if needed
    }

    // Resize the state vector
    x.block(newStateStart, 0, newStateSize, x.cols()) = obsWRF;

    // Transformation matrix (2 x 4)
    Tx(0,0) = 1.0;
    Tx(0,2) = -dyr + dyo;
    Tx(1,1) = 1.0;
    Tx(1,2) = dxr + dxo;

    // Transformation matrix (2 x 2)
    Tz(0,0) = cos( phi );
    Tz(0,1) = -dyo;
    Tz(1,0) = sin( phi );
    Tz(1,1) = dxo;

    // Augment
    R(0,0) = sqrt( m_var_range );
    R(1,1) = sqrt( m_var_bearing );

    // Calculate mu
    Eigen::MatrixXd mu = X.block(0, 0, numStates, vehicleSize) * Tx.transpose();

    // Calculate Sigma
    Eigen::Matrix2d Sigma = Tx * X.block(0, 0, vehicleSize, vehicleSize) * Tx.transpose()
                            + Tz * R * Tz.transpose();

    // Update X with mu and Sigma
    X.block(0, newStateStart, vehicleSize, newStateSize) = mu;
    X.block(newStateStart, 0, newStateSize, vehicleSize) = mu.transpose();
    X.block(newStateStart, newStateStart, newStateSize, newStateSize) = Sigma;

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
