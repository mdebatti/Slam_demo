// CMatlabFileReading.cpp
//
// definition of CMatlabFileReading class
// Marc de Battista - 04-08-2002


#include "CMatlabFileReading.h"
using namespace std;

CMatlabFileReading::CMatlabFileReading( const char* filename )
{
    // open the file
    m_fStream.open( filename );

    // Check if the file was opened successfully
    if( m_fStream.fail() )
    {
        cerr << "File " << filename << " could not be opened" << endl;
        exit(1);
    }

    // set the other member variable
    m_nRows = 0;
    m_nCols = 0;

    // file not yet checked for right format
    // sets the m_nRows and m_nCols member variables if OK
    m_validFile = isFileFormatValid();

    if( !m_validFile )
    {
        cerr << "File """ << filename << """ has not a correct format !" << endl;
        exit(1);
    }

    // memorize the file name
    strcpy( m_filename, filename );
}

double CMatlabFileReading::GetNextDoubleNumber()
{
    double temp;

    if(m_validFile)
        m_fStream >> temp;
    else
        cerr << "In CMatlabFileReading::GetNextDoubleNumber(), file not valid" << endl;

    return temp;
}

void CMatlabFileReading::DisplayStatusInfo()
{
    char status[10];

    // See if the file had the correct matrix format
    m_validFile ? strcpy( status, "valid" ) : strcpy( status, "unvalid" );

    // Write the file info
    cout << "File """ << m_filename << """ status: " << status << endl << endl;;

    // The number of rows and columns
    cout << "Number of extracted rows: " << m_nRows << endl;
    cout << "Number of extracted columns: " << m_nCols << endl << endl;
}


void CMatlabFileReading::DisplayFile()
{
    bool eof = false;
    char byte;

    while( !eof )
    {
        // Read a byte from the stream
        m_fStream.get( byte );

        // Check for the end of the file
        if( !m_fStream.eof() )
            cout << byte;
        else
            eof = true;
    }

    // rewind the pointer and clear the flags
    m_fStream.clear();
    m_fStream.seekg( ios::beg );
}



bool CMatlabFileReading::isFileFormatValid()
{
    bool valid = true;
    bool eof = false;
    unsigned int spaceCounterCurrent = 0;
    unsigned int spaceCounterPrevious = 0;
    unsigned int rowsCounter = 0;
    char byte;

    // read the stream byte by byte and check the
    // formating of the file (characters + matrixity)
    while( !eof )
    {
        // Read a byte from the stream
        m_fStream.get( byte );

        // Check for the end of the file
        if( !m_fStream.eof() )
        {
            // check if the characters present in the file is either a space,
            // a line feed, a number ranging from 0 to 9 or a minus sign
            if( !isOK( byte ) )
            {
                valid = false;
                cout << "the character that crashed: " << byte << endl;
                eof = true;
            }

            // check that the matrix formating is respected
            if( isSP( byte ) )
                spaceCounterCurrent++;

            if( isLF( byte ) )
            {
                // Initialize the number of element ofthe first row
                // and increment rowsCounter
                if( rowsCounter++ == 0 )
                    spaceCounterPrevious = spaceCounterCurrent;

                if( spaceCounterCurrent == spaceCounterPrevious )
                {
                    spaceCounterPrevious = spaceCounterCurrent;
                    spaceCounterCurrent = 0;
                }
                else
                {
                    valid = false;
                    eof = true;
                }
            }
        }
        else
            eof = true;
    }

    if( valid )
    {
        // Check if there was a last line feed or not
        m_fStream.seekg( ios::end );
        m_fStream.get( byte );
        if( !isLF( byte ) )
            rowsCounter++;

        // Set the matrix dimension
        m_nRows = rowsCounter;
        m_nCols = spaceCounterPrevious + 1;
    }

    // rewind the pointer and clear the flags
    m_fStream.clear();
    m_fStream.seekg( ios::beg );

    return valid;
}

void CMatlabFileReading::RewindFile()
{
    // rewind the pointer and clear the flags
    m_fStream.clear();
    m_fStream.seekg( ios::beg );
}

/*
void main()
{
    CMatlabFileReading file("utrue.txt");
    file.DisplayStatusInfo();

    for( int i = 0; i < 12; i++ )
        cout << file.GetNextDoubleNumber() << endl;

    cout << endl;

    for( i = 0; i < 12; i++ )
        cout << file.GetNextDoubleNumber() << endl;
}
*/
