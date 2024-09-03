#ifndef CDoubleVector_04082002
#define CDoubleVector_04082002

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000



#include<iostream>
#include<fstream>
#include<vector>
#include "CDynamicArray.h"
#include "CMatlabFileReading.h"
using namespace std;

#define DEFAULT_RESERVE_BLOCK_INCREMENT 1000	// Space for 1000 elements


typedef vector<CDynamicArray> DOUBLEVECTOR;



class CDoubleVector
{
public:
    CDoubleVector( int nRows, int nCols );
    CDoubleVector( CMatlabFileReading& afile );
    CDoubleVector( const char* fileName );
    CDoubleVector( const int nCols );
    CDoubleVector();
    CDoubleVector( const CDoubleVector& aDoubleVector );

    void AddToTail( const CDynamicArray& array );
    void Resize( int newsize );
    void SetnCols( int nCols ) { m_nCols = nCols; };

    int GetNumCols() { return m_nCols; };
    int GetNumRows() { return m_nRows; };
    DOUBLEVECTOR& GetData() { return m_Data; };

    CDoubleVector& operator=( const CDoubleVector& aDoubleVector );


    void Display();
    void SaveToFile( const char* filename );

    CDynamicArray& operator[]( int k );


    ~CDoubleVector(){ };

    void FillVector( CMatlabFileReading& afile );
    void FillVector( const char* filename );

private:
    DOUBLEVECTOR m_Data;
    int m_nRows;
    int m_nCols;
    bool m_Dynamic;




};





#endif // CDoubleVector_04082002
