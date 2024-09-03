#include "CDoubleVector.h"

//#define VERBOSE



CDoubleVector::CDoubleVector( int nRows, int nCols )
{
    int i=0;

    // In this case, the memory is allocated all at once
    m_Dynamic = false;
    m_nCols = nCols;
    m_nRows = nRows;

    // Do the allocation for the vector object
    m_Data.resize( nRows );

    // and now allocate memory for every vector
    for( i = 0; i < nRows; i++ )
        m_Data[i].ReSize( nCols );

#ifdef VERBOSE
    cout << "CDoubleVector::CDoubleVector( int nRows, int nCols ) called" << endl;
#endif

}

CDoubleVector::CDoubleVector( CMatlabFileReading& afile )
{
    FillVector( afile );

#ifdef VERBOSE
    cout << "CDoubleVector::CDoubleVector( CMatlabFileReading& afile ) called" << endl;
#endif
}


CDoubleVector::CDoubleVector( const CDoubleVector& aDoubleVector )
{
    cout << "CDoubleVector::CDoubleVector( const CDoubleVector aDoubleVector ) called." << endl;
}

CDoubleVector::CDoubleVector( const char* fileName )
{
    CMatlabFileReading file( fileName );
    FillVector( file );
}


CDoubleVector::CDoubleVector( const int nCols )
{
    int i=0;

    // In this case, the memory is allocated all at once
    m_Dynamic = true;
    m_nCols = nCols;
    m_nRows = 0;

#ifdef VERBOSE
    cout << "CDoubleVector::CDoubleVector( int nCols ) called" << endl;
#endif
}

// In VehicleModel, there is 4 CDoubleVector object that will call this constructor
// upon declaration of a VehicleModel object. No means to set the right number of
// columns ?
CDoubleVector::CDoubleVector()
{
    // Initialization
    m_Dynamic = true;
    m_nCols = 0;
    m_nRows = 0;

#ifdef VERBOSE
    cout << "CDoubleVector::CDoubleVector() called" << endl;
#endif

}


CDynamicArray& CDoubleVector::operator[]( int k )
{
    // Is subscript negative
    if( k < 0 )
    {
        cerr << "Index is negative in CDoubleVector::operator[]: k = " << k << endl;
        exit(1);
    }

    // Is it bigger than max size
    if( k >= m_Data.max_size() )
    {
        cerr << "In CDoubleVector::operator[] k is bigger than m_Data.max_size()" << endl;
        exit(1);
    }

    // it means a new element has to be added to the tail of the vector
    if( k > m_Data.size()-1 )
    {
        cerr << "Subscribt out of range in CDoubleVector::operator[]: " << k << "/" << m_Data.size()-1 << endl;
        exit(1);
    }

    return m_Data[k];
}

void CDoubleVector::FillVector( CMatlabFileReading& afile )
{
    int i, j;

#ifdef VERBOSE
    cout << "CDoubleVector::FillVector(" << afile.GetFileName() << ")" << endl;
#endif

    // Initialization
    m_nRows = afile.GetRowDim();
    m_nCols = afile.GetColDim();
    m_Dynamic = false;

    // Do the allocation for the vector object
    m_Data.resize( m_nRows );

    // and now allocate memory for every vector
    for( i = 0; i < m_nRows; i++ )
        m_Data[i].ReSize( m_nCols );

    // just make sure we start up from the beginning of the file
    afile.RewindFile();

    // Fill the vector
    for( i = 0; i < m_nRows; i++ )
        for( j = 0; j < m_nCols; j++ )
            m_Data[i][j] = afile.GetNextDoubleNumber();
}

void CDoubleVector::FillVector( const char* filename )
{
    CMatlabFileReading afile( filename );
    FillVector( afile );
}

void CDoubleVector::AddToTail( const CDynamicArray& array )
{

    m_Data.push_back( array );
    m_nRows++;

#ifdef VERBOSE
    cout << "CDoubleVector::AddToTail( ) called. Size: " << m_Data.size()
        << "   Capacity: " << m_Data.capacity() << endl;
#endif

}

void CDoubleVector::SaveToFile( const char* filename )
{
    int i,j;

    ofstream File( filename );

    File.precision( 20 );

    // Save the vector
    for( i = 0; i < m_nRows; i++ )
    {
        for( j = 0; j < m_nCols-1; j++ )
            File << m_Data[i][j] << " ";

        File << m_Data[i][j] << '\n';
    }

}

void CDoubleVector::Display()
{
    cout << endl;
    cout << "Dimension of vector: " << m_nRows << "x" << m_nCols << endl;
    cout << "Content: " << endl;

    for( int i = 0; i < m_nRows; i++ )
        m_Data[i].DisplayHorizontally();

    cout << endl;
}


// Copy Constructor CDynamicArray::CDynamicArray will be called newsize times
// which means that all the CDynamicArray objects present in the CDoubleVector
// are copyed and then erased, which forbids resizing the CDoubleVector every time
// there is a new element to be added in order to be able to use the [] operator for
// the sake of code consiseness. Use AddTail instead since the CDynamic object being
// added is copyed in the CDoubleVector object
void CDoubleVector::Resize( int newsize )
{
    if( newsize < m_Data.max_size() )
    {
        m_Data.resize( newsize );
        m_nRows = newsize;
    }
    else
    {
        cerr << "In CDoubleVector::Resize( int newsize ): newsize is " << newsize << " and max_size is " << m_Data.max_size() << endl;
        exit(1);
    }
}


CDoubleVector& CDoubleVector::operator=( const CDoubleVector& aDoubleVector )
{
    cout << "CDoubleVector& CDoubleVector::operator=() called." << endl;

    return *this;
}

/*

void main()
{

    /*
    CDoubleVector vector("utrue.txt");
    CDoubleVector vec(3);
    CDynamicArray arr(3);
    arr[0] = 3.12;
    arr[1] =89.9;
    arr[2] = 78.9;


// 	vec[15] = arr; the if instruction in the overloading of = doesn't work - find out!!

    vec.AddToTail(arr);
    vec.AddToTail(arr);
    arr[0] = 63.52;
    arr[1] = 289.9;
    arr[2] = 978.9;
    vec.AddToTail(arr);


    vec.Display();
    vector.Display();

    vector.SaveToFile("utrueMode.txt");

    */

/*
    CDoubleVector vect(3);

    CDynamicArray arr(3);
    arr[0] = 3.12;
    arr[1] =89.9;
    arr[2] = 78.9;
    vect.AddToTail(arr);
        vect.AddToTail(arr);
            vect.AddToTail(arr);
                vect.AddToTail(arr);
    vect.Display();



}

*/
