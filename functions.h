#pragma once

#include "constants.h"
#include "types.h"
#include <cmath>  // For std::abs
#include<fstream>

double normalizeAngle(double angle);

bool isEqualWithTolerance(int a, double b, double tolerance = 1e-9);
std::string generateNewFilename(const std::string& addon_string, const std::string& filename);
void showProgressBar(size_t current, size_t total, size_t barWidth = 50);

template <typename T>
void saveDataToFile(const T& data, const std::string& filename);

template <typename Matrix1, typename Matrix2>
void testEigenMaxtrixEquality(const Matrix1& M1, const Matrix2& M2, const std::string& label_str, const std::string& label_str2);

