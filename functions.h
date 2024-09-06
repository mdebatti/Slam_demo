#pragma once

#include "constants.h"
#include "types.h"
#include <cmath>  // For std::abs

double normalizeAngle(double angle);

bool isEqualWithTolerance(int a, double b, double tolerance = 1e-9);
std::string generateNewFilename(const std::string& addon_string, const std::string& filename);
void showProgressBar(size_t current, size_t total, size_t barWidth = 50);
