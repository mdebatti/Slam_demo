#include "VehicleModel.h"


// First Constructor that loads and allocate all the member variable of the class VehicleModel thanks
// to the file. It is always assumed that one variable is one column of the file
VehicleModel::VehicleModel( const char* xtrueFile, const char* utrueFile, const char* refpathFile, const char* beaconFile, int numLoops )
{
    // Load the true vehicle location, the corresponding controls, the reference file
    // and the beacons in the corresponding member data. The allocation for these arrays
    // is done via CContainerVecor::AllocateArray once the dimensions are extracted from file
    m_xtrue.FillVector( xtrueFile );
    m_utrue.FillVector( utrueFile );
    m_Beacons.FillVector( beaconFile );
    m_RefPath.FillVector( refpathFile );

    // Set the number of loops
    m_NumLoops = numLoops;
}

// Second Constructor that loads and allocate all the member variable of the class VehicleModel thanks
// to the file. It is always assumed that one variable is one column of the file
VehicleModel::VehicleModel( const char* refpathFile, const char* beaconFile, int numLoops)
{

    // Load the true vehicle location, the corresponding controls, the reference file
    // and the beacons in the corresponding member data. The allocation for these arrays
    // is done via CContainerVecor::AllocateArray once the dimensions are extracted from file
    // This works fine, no diffrence between the original matlab file and the CDoubleVector.SaveToFile() version
    m_Beacons.FillVector( beaconFile );
    m_RefPath.FillVector( refpathFile);

    // Set the number of loops
    m_NumLoops = numLoops;

    // Show the number of row and columns extracted from the files
    cout << "Beacons: " << m_Beacons.GetNumRows() << "x" << m_Beacons.GetNumCols() << endl;
    cout << "RefPath: " << m_RefPath.GetNumRows() << "x" << m_RefPath.GetNumCols() << endl;

    // Compute utrue and xtrue
    ComputeControls();

}

// Simulate radar observations from the vehicle between a start location xtrue_start and
// an end position xtrue_end based on the true beacon location and the true vehicle state xtrue.
// When multiple beacons are observed, only the nearest one is recorded.
//
// The location vectors xtrue_start and xtrue_end each consist of a vector of [x,y,phi,t]
// describing the location of the vehicle at a given time.
// The beacon map is an array of locations
//
// ObsWRF are the observation in world reference frame, ObsVRF are in vehicle reference frame and
// the function returns either true or false if it observed something or not


bool VehicleModel::SimObs( const int k, CDynamicArray& ObsWRF, CDynamicArray& ObsRB, int sigma_range, int sigma_bearing )
{
    // Just check if the index is valid ( m_xtrue[k+1] is found in the code, so go to max. k = #element-2 )
    if( ( k < 0 ) || ( k > m_xtrue.GetNumRows() - 2 ) )
    {
        cerr << "In VehicleModel::SimObs(), index is out of range (" << k << "/" << m_xtrue.GetNumRows() - 2 << ")" << endl;
        exit(1);
    }

    // The algorithm now proceeds by finding the two bounding lines from radar to max range.
    // If the beacon lies between these two lines, then it will be observed

    // Cos an Sin of the vehicle's orientation at start and end location
    double c_phi1 = cos( m_xtrue[k][2] );
    double c_phi2 = cos( m_xtrue[k+1][2] );
    double s_phi1 = sin( m_xtrue[k][2] );
    double s_phi2 = sin( m_xtrue[k+1][2] );

    // Find location of the radar at start location, X and Y coordinate in WRF
    double radx1 = m_xtrue[k][0] + (SlamPhysicalConstants::XOFFSET*c_phi1 - SlamPhysicalConstants::YOFFSET*s_phi1 );
    double rady1 = m_xtrue[k][1] + (SlamPhysicalConstants::XOFFSET*s_phi1 + SlamPhysicalConstants::YOFFSET*c_phi1 );

    // Find location and aim of radar at end location, X and Y coordinate in WRF
    double radx2 = m_xtrue[k+1][0] + (SlamPhysicalConstants::XOFFSET*c_phi2 - SlamPhysicalConstants::YOFFSET*s_phi2 );
    double rady2 = m_xtrue[k+1][1] + (SlamPhysicalConstants::XOFFSET*s_phi2 + SlamPhysicalConstants::YOFFSET*c_phi2 );

    // gets normalized angle for the radar aim at the end and start location
    double radphi1 = a_sub( m_xtrue[k][3]*SlamPhysicalConstants::R_RATE, 0 );
    double radphi2 = a_sub( m_xtrue[k+1][3]*SlamPhysicalConstants::R_RATE, 0 );

    // construct radar aiming line segments for checking beacon view
    double dx1 = SlamPhysicalConstants::R_MAX_RANGE*cos( radphi1 + m_xtrue[k][2] );
    double dy1 = SlamPhysicalConstants::R_MAX_RANGE*sin( radphi1 + m_xtrue[k][2] );
    double dx2 = SlamPhysicalConstants::R_MAX_RANGE*cos( radphi2 + m_xtrue[k+1][2] );
    double dy2 = SlamPhysicalConstants::R_MAX_RANGE*sin( radphi2 + m_xtrue[k+1][2] );

    // To be observed, the beacon must lie between these two lines the test is simply that the dot
    // products are positive and negative respectively. The nearest beacon within max range is recorded.

    double bRange	= SlamPhysicalConstants::R_MAX_RANGE;			// Initialization to the largest possible range
    double bIndex	= 0;					// If it remains null, no beacon observed
    for(int i = 0; i < m_Beacons.GetNumRows(); i++ )
    {
        double dx = m_Beacons[i][0] - radx1;
        double dy = m_Beacons[i][1] - rady1;
        double r = sqrt( dx*dx + dy*dy );
        double d1 = ( (m_Beacons[i][1] - rady1)*dx1 ) - ( (m_Beacons[i][0] - radx1)*dy1 );
        double d2 = ( (m_Beacons[i][1] - rady2)*dx2 ) - ( (m_Beacons[i][0] - radx2)*dy2 );
        if( ( r < SlamPhysicalConstants::R_MAX_RANGE ) && ( d1 > 0 ) && ( d2 < 0 ) && ( r < bRange ) )
        {
            bRange = r;
            bIndex = i;
        }
    }

    // if a beacon is found, assume it was seen from xtrue_start:
    if( bIndex )
    {
        double phi = a_sub( m_xtrue[k][2], 0);
        ObsRB[0] = bRange;
        ObsRB[1] = a_sub( atan2( m_Beacons[bIndex][1] - rady1 , m_Beacons[bIndex][0] - radx1 ), phi );
        ObsRB[2] = bIndex; 				// beacon index
        ObsRB[3] = m_xtrue[k][3]; 		// time stamp

        // add noise noise model
        ObsRB[0] = ObsRB[0]; // + aEKF.GetSigmaRange()*rand(1);
        ObsRB[1] = ObsRB[1]; // + aEKF.GetSigmaBearing()*randn(1);

        // observation in world reference frame
        ObsWRF[0] = radx1 + ObsRB[0]*cos( ObsRB[1] + m_xtrue[k][2]);
        ObsWRF[1] = rady1 + ObsRB[0]*sin( ObsRB[1] + m_xtrue[k][2]);
        ObsWRF[2] = bIndex; 				// beacon index
        ObsWRF[3] = m_xtrue[k][3]; 			// time stamp

        return true;
    }
    else
    {
        ObsRB[0] = 0.0;
        ObsRB[1] = 0.0;
        ObsRB[2] = 0.0;
        ObsRB[3] = m_xtrue[k][3];

        // observation in world reference frame
        ObsWRF[0] = 0.0;
        ObsWRF[1] = 0.0;
        ObsWRF[2] = 0.0;
        ObsWRF[3] = m_xtrue[k][3];

        return false;
    }
}



void VehicleModel::GetError( const int k, double& PositionError, double& OrientationError )
{
    // A point from the reference path is picked out as being the reference
    // point if it is the closed to the current position at time k. The search
    // is made within a certain range of the reference path, in order to spare
    // computationnal resources
    double dmin;
    double dpx, dpy, dvx, dvy, dx, dy, d, phiest;
    int i;
    static int matchingIndex;	// To which point in the path the actual position is refered to
    int startIndex, endIndex, swapIndex;	// Define the search region in the path
    int numPathPoint = m_RefPath.GetNumRows();

    // matching index starts with 0 and since it's declared as static,
    // it correspond to the previously matched point.
    startIndex = matchingIndex - SlamConstants::PATHWINDOW;
    endIndex   = matchingIndex + SlamConstants::PATHWINDOW;

    // If the end index is bigger that the number of point in the reference
    // path array, divide this index by the number of element. The rest gives
    // the new end index since the path is assumed circular if m_NumLoops is
    // bigger than one
    if( endIndex > numPathPoint-1 )
    {
        if( m_NumLoops > 1 )
        {
            endIndex = endIndex % numPathPoint;
        }
        else
        {
            endIndex = numPathPoint-1;
        }
    }

    // It's about the same for the starting index
    if( startIndex < 0 )
    {
        if( m_NumLoops > 1 )
        {
            startIndex = numPathPoint + startIndex;
        }
        else
        {
            startIndex = 0;
        }
    }

    // Ensure that endIndex is always bigger that start index
    if( startIndex > endIndex )
    {
        swapIndex = endIndex;
        endIndex = startIndex;
        startIndex = swapIndex;
    }

    dmin = SlamPhysicalConstants::WORLD_SIZE;

    // Find the point of the path that is refered to the actual position
    for( i = startIndex; i < endIndex; i++)
    {
        dx = m_RefPath[i][0] - m_xtrue[k][0];
        dy = m_RefPath[i][1] - m_xtrue[k][1];
        d  = sqrt( dx*dx + dy*dy );

        if( d <= dmin )
        {
            dmin = d;
            matchingIndex = i;
        }
    }


    dpx = m_RefPath[matchingIndex+1][0] - m_RefPath[matchingIndex][0];
    dpy = m_RefPath[matchingIndex+1][1] - m_RefPath[matchingIndex][1];

    dvx = m_xtrue[k][0] - m_RefPath[matchingIndex][0];
    dvy = m_xtrue[k][1] - m_RefPath[matchingIndex][1];

    // Position error
    PositionError = dpy*dvx - dpx*dvy;

    // Estimated angle to be reached
    phiest = atan2( dpy, dpx);

    // Orientation error
    OrientationError = a_sub(phiest, m_xtrue[k][2]);

}

void VehicleModel::GetControl( const int k, double perr, double oerr )
{
    // Declaration of two CDynamicArray object to hold the controls and
    // state vector that has to be added to the CDoubleVector member variable
    // of the current VehicleModel object. More efficient if declared as static ????
    // since this function is called for every element of the CDoubleVector::m_Data, which
    // is typically around 10'000 times
    CDynamicArray utrue_next( SlamArraySize::VEHICLE_CONTROLS_DIM );
    CDynamicArray xtrue_next( SlamArraySize::VEHICLE_PATH_DIM );

    // The control vector utrue = [v, steer, time]'
    utrue_next[0] = m_utrue[k][0];
    utrue_next[1] = a_add( SlamConstants::KP*perr, SlamConstants::KO*oerr);
    utrue_next[2] = m_utrue[k][2] + SlamConstants::DT;


    // The true state vector of the vehicle xtrue = [x,y,theta,dt] dt or R ???!!! check it here !!!!???
    xtrue_next[0] = m_xtrue[k][0] + SlamConstants::DT*utrue_next[0]*cos( m_xtrue[k][2] + utrue_next[1] );
    xtrue_next[1] = m_xtrue[k][1] + SlamConstants::DT*utrue_next[0]*sin( m_xtrue[k][2] + utrue_next[1] );
    xtrue_next[2] = m_xtrue[k][2] + SlamConstants::DT*utrue_next[0]*sin( utrue_next[1] ) / SlamPhysicalConstants::WHEEL_BASE;
    xtrue_next[3] = m_xtrue[k][3] + SlamConstants::DT;

    // Add the new controls and state vector to the corresponding CDoubleArray member data
    m_utrue.AddToTail( utrue_next );
    m_xtrue.AddToTail( xtrue_next );
}

void VehicleModel::CalcSingleStepVehicleState( const int k, double perr, double oerr )
{
    // Declaration of two CDynamicArray object to hold the controls and
    // state vector that has to be added to the CDoubleVector member variable
    // of the current VehicleModel object. More efficient if declared as static ????
    // since this function is called for every element of the CDoubleVector::m_Data, which
    // is typically around 10'000 times
    CDynamicArray xtrue_next( SlamArraySize::VEHICLE_PATH_DIM );

    // The control vector utrue = [v, steer, time]'
    utrue_next[0] = m_utrue[k][0];
    utrue_next[1] = a_add( SlamConstants::KP*perr, SlamConstants::KO*oerr);
    utrue_next[2] = m_utrue[k][2] + SlamConstants::DT;


    // The true state vector of the vehicle xtrue = [x,y,theta,dt] dt or R ???!!! check it here !!!!???
    xtrue_next[0] = m_xtrue[k][0] + SlamConstants::DT*utrue_next[0]*cos( m_xtrue[k][2] + utrue_next[1] );
    xtrue_next[1] = m_xtrue[k][1] + SlamConstants::DT*utrue_next[0]*sin( m_xtrue[k][2] + utrue_next[1] );
    xtrue_next[2] = m_xtrue[k][2] + SlamConstants::DT*utrue_next[0]*sin( utrue_next[1] ) / SlamPhysicalConstants::WHEEL_BASE;
    xtrue_next[3] = m_xtrue[k][3] + SlamConstants::DT;

    // Add the new controls and state vector to the corresponding CDoubleArray member data
    m_utrue.AddToTail( utrue_next );
    m_xtrue.AddToTail( xtrue_next );
}

void VehicleModel::ComputeControls()
{

    // position and orientation error of the vehicle by respect
    // to the reference path. Calculated in member function GetError()
    // and used by member function get control
    double perr, oerr;

    // Used to detect the vehicle's comming close to its start and thus
    // break if it loopCount is equal to NumLoop. dx and dy are the x and
    // y distance between the vehicle and the starting point ofthe path
    double old_d, new_d;
    double dx, dy;
    int loopCount, k;
    bool finished;

    // Initialize the number of columns (or # of variables) of the
    // m_xtrue and m_utrue CDoubleVector object
    m_xtrue.SetnCols( SlamArraySize::VEHICLE_PATH_DIM );
    m_utrue.SetnCols( SlamArraySize::VEHICLE_CONTROLS_DIM );

    // For the initial condition
    CDynamicArray utrue_init( SlamArraySize::VEHICLE_CONTROLS_DIM );
    CDynamicArray xtrue_init( SlamArraySize::VEHICLE_PATH_DIM );

    // Initialisation with the initial position of the vehicle taken from
    // the reference path xtrue_init = [ X, Y, Theta, Time ]
    xtrue_init[0] = m_RefPath[0][0];
    xtrue_init[1] = m_RefPath[0][1];
    xtrue_init[2] = atan2( m_RefPath[1][1] - m_RefPath[0][1], m_RefPath[1][0] - m_RefPath[0][0] );
    xtrue_init[3] = 0;

    // Initialization of the control vector utrue_init = [ Velocity, steering angle, Time]
    utrue_init[0] = SlamConstants::VVEL;
    utrue_init[1] = 0;
    utrue_init[2] = 0;

    // Start the CDoubleVector Object with the initial CDynamicArray object
    // the CDoubleVector copy constructor is used to fill the CDoubleVector
    // member data
    m_xtrue.AddToTail( xtrue_init );
    m_utrue.AddToTail( utrue_init );

    // Initialization of the loop's variables
    loopCount = 0;
    new_d = 0;
    k = 0;
    finished = false;

    // Control Loop - The true path and true input are stocked in xtrue and utrue
    while( !finished )
    {
        // At time k, get the error between actual position
        // and reference path
        GetError( k, perr, oerr );

        // And use it to compute the new control vector for time k+1 along
        // with the new position for time k+1. This functions internally
        // adds these CDynamicArray to the tail of the m_utrue and m_xtrue
        // CDoubleVector member data
        GetControl( k, perr, oerr );

        // if the vehicle arrives close to the end, increment the
        // loop counter until equal to NumLoop and then break
        old_d = new_d;
        dx = ( m_xtrue[k+1][0] - m_RefPath[0][0] );
        dy = ( m_xtrue[k+1][1] - m_RefPath[0][1] );
        new_d = sqrt( dx*dx + dy*dy );

        // Getting really close to the start
        if( new_d < SlamConstants::DISTANCE_GAP_FOR_CLOSING_LOOP )
        {
            // There is actually a range of points where new_d is less than
            // 0.2 when the vehicle is close to the startand the following
            // trick avoids incrementing loopCount too many times
            if( old_d > new_d )
                loopCount++;

            new_d=0;

            // If the number of loops is done, stop
            finished = (loopCount == m_NumLoops) ? true : false;
        }

        // increment index variable k to process the next element
        k++;
    }

    // Convert the vehicle speed in rad/s
    for( k = 0; k < m_utrue.GetNumRows(); k++ )
        m_utrue[k][0] = m_utrue[k][0]/SlamPhysicalConstants::WHEEL_RADIUS;

    m_utrue.SaveToFile("utrue_large_5Loops_cpp.txt");
    m_xtrue.SaveToFile("xtrue_large_5Loops_cpp.txt");
}

#ifdef TEST_VM
void main()
{
    // Test the VehicleModel::SimObs() member function. Loads all the needed values from text files
    // generated with matlab. The result is saved to text file so that it can be compared to the values
    // generated by the reference matlab code
#ifdef TEST_SIMOBS

    int k;

    CDynamicArray obsRB( RB_OBS_DIM );		// The observation in range and bearing coordinate
    CDynamicArray obsWRF( WRF_OBS_DIM );	// ... in the world reference frame system

    CDoubleVector ObsRB_vect( RB_OBS_DIM );	// To store all the CDynamicArray object
    CDoubleVector ObsWRF_vect( WRF_OBS_DIM );	// idem

    // Load the values from the files in the variables
    VehicleModel VM( "RefPath_large_5Loops.txt", "Beacons_large_5Loops.txt",5);

    for( k = 0; k < VM.GetXtrueArrayDim()-1; k++ )
    {
        // Simulates the Observation
        VM.SimObs( k, obsWRF, obsRB );

        // Store them in a CDoubleVEctor object
        ObsRB_vect.AddToTail( obsRB );
        ObsWRF_vect.AddToTail( obsWRF );
    }

    // ... and save them to file
//	ObsWRF_vect.SaveToFile("ObsWRF_large_5Loops_cpp.txt");
//	ObsRB_vect.SaveToFile("ObsRB_large_5Loops_cpp.txt");
#endif

//	for ( k = 0; k < 10; k++ )
//		VM.GetXtrue(k).DisplayHorizontally();

}
#endif
