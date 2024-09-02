#pragma once

#include<iostream>
#include<fstream>
#include<cstring>
using namespace std;

#define SPACE			0x20
#define LINE_FEED		0x0A
#define NINE			0x39
#define ZERO			0x30
#define MINUS			0x2D
#define POINT			0x2E




class CMatlabFileReading
{
public:
    CMatlabFileReading( const char* filename );
    void DisplayStatusInfo();
    void DisplayFile();
    int GetRowDim() const { return m_nRows; };
    int GetColDim() const { return m_nCols; };
    double GetNextDoubleNumber();
    void RewindFile();
    const char* GetFileName() const { return m_filename; };

private:
    bool isFileFormatValid();
    bool isLF( char byte ) const { return byte == LINE_FEED ?  true : false; };
    bool isSP( char byte ) const { return byte == SPACE ? true : false; };
    bool isMinus( char byte ) const { return byte == MINUS ? true : false; };
    bool isNum( char byte ) const { return ((byte <= NINE)&&(byte >= ZERO)) ? true : false;};
    bool isPoint( char byte ) const { return byte == POINT ?  true : false; };
    bool isOK( char byte) const { return (isLF(byte) || isSP(byte) || isMinus(byte) || isNum(byte) || isPoint(byte)) ? true : false;};


    ifstream m_fStream;
    int m_nRows;
    int m_nCols;
    bool m_validFile;
    char m_filename[256];
};
