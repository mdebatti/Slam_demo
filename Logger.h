#pragma once

#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include "types.h"

class Logger {
public:
    explicit Logger(const std::string& filename);
    ~Logger();

    void log(int k, const std::string& message);
    void setStep(int step);

    template <typename MatrixType>
    void logMatrix(int k, const std::string& label, const MatrixType& matrix);

    template <typename VectorType>
    void logVector(int k, const std::string& label, const VectorType& vector);

private:
    std::ofstream _logFile;
    int _currentStep;   // To keep track of the simulation step
    int _lastStep;      // To introduce a line break
};
