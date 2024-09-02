// DynamicArray.cpp: implementation of the CDynamicArray class.
//
//////////////////////////////////////////////////////////////////////

#include "CDynamicArray.h"
//#define VERBOSE

CDynamicArray::CDynamicArray( int size ):m_Size( size ), m_constructed( CONSTRUCTION_CONST)
{
    // Allocate dynamically the memory for the array
    m_Array = new double[ m_Size ];

#ifdef VERBOSE
    cout << "Constructor CDynamicArray( int size ) called: 0x" << m_Array << endl;
#endif
}

CDynamicArray::CDynamicArray( ):m_Size( 0 ), m_constructed( CONSTRUCTION_CONST)
{
    // Allocate dynamically the memory for the array
    m_Array = new double[ m_Size ];
    if( m_Array == NULL )
    {
        cerr << "CDynamicArray::CDynamicArray( ), memory could not be allocated" << endl;
        exit(1);
    }


#ifdef VERBOSE
    cout << "Default Constructor CDynamicArray() called: Ox" << m_Array << endl;
#endif
}


CDynamicArray::~CDynamicArray()
{
#ifdef VERBOSE
    cout << "Destructor CDynamicArray called: Ox" << m_Array << endl;
#endif

    // Free the memory
    delete [] m_Array;



}

void CDynamicArray::ReSize( int size )
{
    int i;
    int oldSize;

//	char Message[256];

#ifdef VERBOSE
    cout << "CDynamicArray::ReSize called: 0x" << m_Array << " is changed to 0x";
#endif

    if( size > 0 )
    {
        // keep the old adress and array size
        double* m_ArrayOld = m_Array;

        // m_Size can be undefined if ReSize called just vector::reserve()
        // since it actually does not call any constructor !!
        oldSize = (m_constructed == CONSTRUCTION_CONST) ? m_Size : 0;

        // Set the member data to the new size
        m_Size = size;

        // declare the extended or truncated table
        m_Array = new double[ m_Size ];
        if( m_Array == NULL )
        {
            cerr << "CDynamicArray::ReSize( int size ), memory could not be allocated" << endl;
            exit(1);
        }

        // Fill it with the old data
        for( i = 0; i < oldSize; i++ )
            m_Array[i] = m_ArrayOld[i];

        // and pad the rest with zeros
        for( i = oldSize; i < m_Size; i++ )
            m_Array[i] = 0;

        // free the memory of the old vector only if it was allocated
        // during construction, otherwise crash
        if( m_constructed == CONSTRUCTION_CONST )
            delete [] m_ArrayOld;
    }
#ifdef VERBOSE
    cout << m_Array << endl;
#endif
}

CDynamicArray::CDynamicArray( const CDynamicArray& aObj ):m_Size( aObj.m_Size ), m_constructed( CONSTRUCTION_CONST)
{
    int i;


    // Allocate dynamically the memory for the array
    m_Array = new double[ m_Size ];
    if( m_Array == NULL )
    {
        cerr << "In CDynamicArray::CDynamicArray( const CDynamicArray& aObj ), memory could not be allocated" << endl;
        exit(1);
    }

    // Copy the table
    for( i = 0; i < m_Size; i++ )
        m_Array[i] = aObj.m_Array[i];

#ifdef VERBOSE
    cout << "Copy Constructor CDynamicArray::CDynamicArray called: 0x" << m_Array << endl;
#endif
}

void CDynamicArray::DisplayVertically()
{
    for( int i = 0; i < m_Size; i++ )
        cout << m_Array[i] << endl;
}

void CDynamicArray::DisplayHorizontally()
{
    for( int i = 0; i < m_Size; i++ )
        cout << m_Array[i] << " ";

    cout << endl;
}

void CDynamicArray::DisplayMemberDataStatus()
{
    cout << "Object of class CDynamicArray:" << endl;
    for( int i = 0; i < m_Size; i++ )
        cout << m_Array[i] << endl;
    cout << "Adress: " << &m_Array[0] << endl;
    cout << endl;
}

double& CDynamicArray::operator[]( int k )
{
    if( (k < 0) || ( k >= m_Size ) )
    {
        cerr << "The index k (" << k << ") is out of range (0 - " << m_Size-1 << ")" << endl;
        exit(1);
    }
    return m_Array[k];
}

CDynamicArray& CDynamicArray::operator=( const CDynamicArray& aArray )
{
#ifdef VERBOSE
    cout << "CDynamicArray::operator= called" << endl;
#endif

    int i;

    if( this == &aArray )
        return *this;

    if( m_Size != aArray.m_Size )
    {
        // If the dimensions don't match, free the memory and relocate
        m_Size = aArray.m_Size;
        delete [] m_Array;
        m_Array = new double[ m_Size ];
    }

    // Copy element by element
    for( i = 0; i < m_Size; i++ )
        this->m_Array[i] = aArray.m_Array[i];

    return *this;


}

/*
void main()
{
/*
    CDynamicArray a(5);
    CDynamicArray b(3);
    CDynamicArray e;


    a[0] = 0.12;
    a[1] = 0.78;
    a[2] = 0.98;
    a[3] = 3.45;
    a[4] = 9.89;

    b[0] = 0.98;
    b[1] = 99.98;
    b[2] = 65587.7;

    CDynamicArray c(a);

    CDynamicArray d = b;

    a.DisplayMemberDataStatus();
    b.DisplayMemberDataStatus();
    c.DisplayMemberDataStatus();
    d.DisplayMemberDataStatus();

    b = a;
cout << "Assign" << endl;
    b.DisplayMemberDataStatus();
    a.DisplayMemberDataStatus();

    e.DisplayMemberDataStatus();

    a.ReSize(7);
    a.DisplayMemberDataStatus();
*/

    /*
CDynamicArray h;
h.ReSize(7);

h.DisplayMemberDataStatus();
*/
/*
    vector<CDynamicArray> z;

    z.resize(3);

    for( int i = 0; i < 3; i++ )
        z[i].ReSize(5);



}
*/
