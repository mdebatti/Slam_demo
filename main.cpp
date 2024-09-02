#include "constants.h"
#include "KalmanFilter.h"
//#include "Functions.h"
//#include "DynamicArray.h"
#include "VehicleModel.h"

//#include <vector>

//#include <crtdbg.h>			// To check stack status and problems, not include in release version
//#include <afxtempl.h> //CList

#include <stdlib.h>     /* used for EXIT_SUCCESS */
#include <Eigen/Dense>

int main(void)
{
    try{

    CKalmanFilter aEKF;

    // why are RB_OBS_DIM and WRF_OBS_DIM equal to 4?
    CDynamicArray obsRB(RB_OBS_DIM);		// The observation in range and bearing coordinate
    CDynamicArray obsWRF(WRF_OBS_DIM);      // ... in the world reference frame system

    // Load the reference path and the beacons from file, compute the true control vector and
    // the true location of the vehicle
    const char* file_refpath = "../cpp/RefPath_large_5Loops.txt";
    const char* file_beacons = "../cpp/Beacons_small_5Loops.txt";
    int num_loops = 5;
    CVehicleModel VModel(file_refpath, file_beacons, num_loops);

    // The corrupted control vector
    Eigen::VectorXd u = Eigen::VectorXd::Zero(U_CONTROLS_DIM);  // 4x1 vector of zeros

    int k;

    // Initialise the state vector x
    x(0) = VModel.GetXtrue(0)(0);
    x(1) = VModel.GetXtrue(0)(1);
    x(2) = VModel.GetXtrue(0)(2);
    x(3) = SlamPhysicalConstants::WHEEL_RADIUS;


    // Initialise the state covariance matrix X
    X(0,0) = SlamNoise::VAR_XX;
    X(1,1) = SlamNoise::VAR_YY;
    X(2,2) = SlamNoise::VAR_TT;
    X(3,3) = SlamNoise::VAR_RR;

#ifndef HELLO
    k = 0;
    while( k < VModel.GetUtrueArrayDim()-1 )
    {
        // corrupt the input with the current modeled noise
        aEKF.CorruptControls(VModel.GetUtrue(k), u );

//ccc		::cout << u << endl;

        // Prediction Step: the last estimate in fed into the
        // prediction model along with the controls that bring
        // the vehicle to the new position after displacement
        aEKF.PredictState(u);
//		x_pred[0] = x(0);
//		x_pred[1] = x(1);
//		x_pred[2] = x(2);
//		x_pred[3] = x(3);

//	x_List.AddToTail(x_pred);

        // Do an observation from the true location but report it from the predicted one ???
        // if something observed, match it and update the EK

#ifndef DEGH
        if( VModel.SimObs(k, obsWRF, obsRB ) )
        {
            Eigen::VectorXd range_bearing = Eigen::VectorXd::Zero(RB_OBS_DIM);

            for(size_t ii = 0; ii < RB_OBS_DIM; ii++)
            {
                range_bearing(ii) = obsRB[ii];
            }

            // Update the state and covariance matrix if an observation of an already
            // mapped landmark is made. Otherwise, the landmark has not been observed yet
            // and the state vector must be augmented. The matching should be implemented with
            // a manhalobis criterion or a joint compatibility test but here, we just set up
            // a list with the tags of the mapped beacons, which enables to tell if mapped or not
            if( aEKF.isMapped(range_bearing ) )
                aEKF.UpdateEKF(range_bearing );
            else
                aEKF.AddState(range_bearing );
        }

        // store all the state x and its covariance matrix in a list object
        // since the sizes are variable

#endif
//		x_list.push_back( x );
//		X_list.push_back( X );
        k++;
    }

//	x_List.SaveToFile("x_predicted.txt");
#endif

    }//try
    catch(const std::exception &ex)
    {
        std::cout << ex.what() << endl;
    }

      return(EXIT_SUCCESS);
}

