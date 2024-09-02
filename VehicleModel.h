#pragma once

#include <iostream>
#include <fstream>
#include <math.h>
#include "constants.h"
#include "functions.h"
#include "CDoubleVector.h"
#include "CDynamicArray.h"
#include <Eigen/Dense>
using namespace std;




//void SaveToFile( CContainerVector& data );
// void ReadFromFile( CContainerVector& data );

class CVehicleModel
{
public:
    CVehicleModel( const char* xtrueFile, const char* utrueFile, const char* refpathFile, const char* beaconFile, int numLoops = 1);
    CVehicleModel( const char* refpathFile, const char* beaconFile,  int numLoops = 1 );

    bool SimObs( const int k, CDynamicArray& ObsWRF, CDynamicArray& obsRB,  int sigma_range = 0, int sigma_bearing = 0);
    void ComputeControls();
    Eigen::VectorXd GetXtrue( int k )
    {
        Eigen::VectorXd x_true = Eigen::VectorXd::Zero(m_xtrue.GetNumCols());
        for (size_t ii = 0; ii < m_xtrue.GetNumCols(); ++ii)
        {
            x_true(ii) = m_xtrue[k][ii];
        }
        return x_true;
    };

    Eigen::VectorXd GetUtrue( int k )
    {
        Eigen::VectorXd u_true = Eigen::VectorXd::Zero(m_utrue.GetNumCols());
        for (size_t ii = 0; ii < m_utrue.GetNumCols(); ++ii)
        {
            u_true(ii) = m_utrue[k][ii];
        }
        return u_true;
    };

    Eigen::VectorXd GetBeacon( int k )
    {
        Eigen::VectorXd beacons = Eigen::VectorXd::Zero(m_Beacons.GetNumCols());
        for (size_t ii = 0; ii < m_Beacons.GetNumCols(); ++ii)
        {
            beacons(ii) = m_Beacons[k][ii];
        }
        return beacons;
     };

    int GetUtrueArrayDim()  { return m_utrue.GetNumRows(); }; // number of data point in the text file
    int GetXtrueArrayDim()  { return m_xtrue.GetNumRows(); };
    int GetRefPathArrayDim()  { return m_RefPath.GetNumRows(); };
    int GetBeaconsArrayDim()  { return m_Beacons.GetNumRows(); };


private:
    void GetError( const int k, double& PositionError, double& OrientationError );
    void GetControl( const int k, double perr, double oerr );


    CDoubleVector m_xtrue;
    CDoubleVector m_utrue;
    CDoubleVector m_RefPath;
    CDoubleVector m_Beacons;

    int m_NumLoops;


};
