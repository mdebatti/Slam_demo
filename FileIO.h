#pragma once

#include<iostream>
#include<fstream>
#include<cstring>
#include "types.h"
using namespace std;

#define SPACE			0x20
#define LINE_FEED		0x0A
#define NINE			0x39
#define ZERO			0x30
#define MINUS			0x2D
#define POINT			0x2E


class FileIO
{
public:
    FileIO( const char* filename );
    void DisplayStatusInfo();
    void DisplayFile();
    int getRowDim() const { return _nRows; };
    int getColDim() const { return _nCols; };
    double getNextDoubleNumber();
    void rewindFile();
    const char* getFileName() const { return _filename; };
    DataMatrix getDataMatrix(int expected_num_cols);

private:
    bool isFileFormatValid();
    bool isLF( char byte ) const { return byte == LINE_FEED ?  true : false; };
    bool isSP( char byte ) const { return byte == SPACE ? true : false; };
    bool isMinus( char byte ) const { return byte == MINUS ? true : false; };
    bool isNum( char byte ) const { return ((byte <= NINE)&&(byte >= ZERO)) ? true : false;};
    bool isPoint( char byte ) const { return byte == POINT ?  true : false; };
    bool isOK( char byte) const { return (isLF(byte) || isSP(byte) || isMinus(byte) || isNum(byte) || isPoint(byte)) ? true : false;};


    ifstream _fStream;
    int _nRows = 0;
    int _nCols = 0;
    bool _validFile;
    char _filename[256];
};
