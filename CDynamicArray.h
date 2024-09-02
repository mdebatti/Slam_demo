// DynamicArray.h: interface for the CDynamicArray class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_DYNAMICARRAY_H__4B160C2F_FA20_4458_9C5C_2CEABCC34995__INCLUDED_)
#define AFX_DYNAMICARRAY_H__4B160C2F_FA20_4458_9C5C_2CEABCC34995__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include<iostream>
#include<vector>
using namespace std;

#define CONSTRUCTION_CONST 788990	// just to make sure the memory has been allocated properly

class CDynamicArray
{
public:
    CDynamicArray( int size );
    CDynamicArray( const CDynamicArray& aObj );
    ~CDynamicArray();
    void DisplayVertically();
    void DisplayHorizontally();
    void DisplayMemberDataStatus();
    void ReSize( int size );

    double& operator[]( int k );
    CDynamicArray& operator=( const CDynamicArray& aArray );
    CDynamicArray();
private:

    const int m_constructed;
    double* m_Array;
    int m_Size;			// Number of elements, cannot be const in case of = with bigger


};

#endif // !defined(AFX_DYNAMICARRAY_H__4B160C2F_FA20_4458_9C5C_2CEABCC34995__INCLUDED_)

