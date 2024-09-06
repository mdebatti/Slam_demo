#include "FileIO.h"


FileIO::FileIO(const char* filename)
{
    // open the file
    _fStream.open(filename);

    // Check if the file was opened successfully
    if(_fStream.fail())
    {
        cerr << "File " << filename << " could not be opened" << endl;
        exit(1);
    }

    // file not yet checked for right format
    _validFile = isFileFormatValid();

    if(!_validFile)
    {
        cerr << "File """ << filename << """ has not a correct format !" << endl;
        exit(1);
    }

    // Ensure the destination buffer is properly sized to handle the string and null-terminator.
    strncpy(_filename, filename, sizeof(_filename) - 1);
     _filename[sizeof(_filename) - 1] = '\0';

     // just make sure we start up from the beginning of the file at the next read operation
     rewindFile();
}

DataMatrix FileIO::getDataMatrix(int expected_num_cols)
{
    // check number of rows consistent with expectations
    if (_nCols != expected_num_cols)
    {
        cerr << "File """ << _filename << """ was expected to have " <<
                expected_num_cols << " columns but it has " << _nCols << "!" << endl;
        exit(1);
    }

    DataMatrix data;

    // Do the allocation for the vector object
    data.resize(_nRows);

    for(int rr = 0; rr < _nRows; ++rr)
    {
        data[rr].resize(_nCols);
    }

    // Fill the vector
    for(int rr = 0; rr < _nRows; ++rr)
    {
        for(int cc = 0; cc < _nCols; ++cc)
        {
            double value_from_file = getNextDoubleNumber();
            data[rr][cc] = value_from_file;
        }
    }

    return data;
}

double FileIO::getNextDoubleNumber()
{
    double temp = 0.0;

    if(_validFile)
    {
        if (_fStream >> temp)  // Read the next double value
        {
            //std::cout << "Read value: " << temp << std::endl;
        }
        else
        {
            //cerr << "Failed to read the next double from the file." << endl;
        }
    }
    else
    {
        //cerr << "In FileIO::getNextDoubleNumber(), file not valid" << endl;
    }

    return temp;
}

void FileIO::DisplayStatusInfo()
{
    char status[10];

    // See if the file had the correct matrix format
    _validFile ? strcpy(status, "valid") : strcpy(status, "unvalid");

    // Write the file info
    cout << "File """ << _filename << """ status: " << status << endl << endl;;

    // The number of rows and columns
    cout << "Number of extracted rows: " << _nRows << endl;
    cout << "Number of extracted columns: " << _nCols << endl << endl;
}


void FileIO::DisplayFile()
{
    bool eof = false;
    char byte;

    while(!eof)
    {
        // Read a byte from the stream
        _fStream.get(byte);

        // Check for the end of the file
        if( !_fStream.eof() )
        {
            cout << byte;
        }
        else
        {
            eof = true;
        }
    }

    // rewind the pointer and clear the flags
    _fStream.clear();
    _fStream.seekg( ios::beg );
}



bool FileIO::isFileFormatValid()
{
    bool valid = true;
    bool eof = false;
    unsigned int spaceCounterCurrent = 0;
    unsigned int spaceCounterPrevious = 0;
    unsigned int rowsCounter = 0;
    char byte;

    // read the stream byte by byte and check the
    // formating of the file (characters + matrixity)
    while(!eof)
    {
        // Read a byte from the stream
        _fStream.get(byte);

        // Check for the end of the file
        if(!_fStream.eof())
        {
            // check if the characters present in the file is either a space,
            // a line feed, a number ranging from 0 to 9 or a minus sign
            if(!isOK( byte ))
            {
                valid = false;
                cout << "the character that crashed: " << byte << endl;
                eof = true;
            }

            // check that the matrix formating is respected
            if(isSP( byte ))
            {
                spaceCounterCurrent++;
            }

            if(isLF( byte ))
            {
                // Initialize the number of element ofthe first row
                // and increment rowsCounter
                if(rowsCounter++ == 0)
                {
                    spaceCounterPrevious = spaceCounterCurrent;
                }

                if(spaceCounterCurrent == spaceCounterPrevious)
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
        {
            eof = true;
        }
    }

    if(valid)
    {
        // Check if there was a last line feed or not
        _fStream.seekg(ios::end);
        _fStream.get(byte);
        if( !isLF(byte ))
            rowsCounter++;

        // Set the matrix dimension
        _nRows = rowsCounter;
        _nCols = spaceCounterPrevious + 1;
    }

    // rewind the pointer and clear the flags
    _fStream.clear();
    _fStream.seekg(ios::beg);

    return valid;
}

void FileIO::rewindFile()
{
    // rewind the pointer and clear the flags
    _fStream.clear();
    _fStream.seekg(ios::beg);
}

/*
void main()
{
    FileIO file("utrue.txt");
    file.DisplayStatusInfo();

    for( int i = 0; i < 12; i++ )
        cout << file.GetNextDoubleNumber() << endl;

    cout << endl;

    for( i = 0; i < 12; i++ )
        cout << file.GetNextDoubleNumber() << endl;
}
*/
