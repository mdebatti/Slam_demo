#include "functions.h"


double normalizeAngle(double angle)
{
    // Normalize the angle to be within the range -PI to PI
    angle = fmod(angle + SLAM_CONST::PI, 2.0 * SLAM_CONST::PI);

    if (angle < 0)
    {
        angle += 2.0 * SLAM_CONST::PI;
    }

    return angle - SLAM_CONST::PI;
}

bool isEqualWithTolerance(int a, double b, double tolerance)
{
    return std::abs(a - b) < tolerance;
}

std::string generateNewFilename(const std::string& addon_string, const std::string& filename)
{
    // Step 1: Extract the filename without the path
    size_t last_slash_pos = filename.find_last_of('/');
    std::string basename = (last_slash_pos == std::string::npos) ? filename : filename.substr(last_slash_pos + 1);

    // Step 2: Remove the ".txt" extension
    size_t extension_pos = basename.find(".txt");
    std::string name_part = (extension_pos == std::string::npos) ? basename : basename.substr(0, extension_pos);

    // Step 3: Concatenate the string "_vehicle_controls_clean" with the extracted filename
    std::string new_string = addon_string + "_" + name_part;

    // Step 4: Remove any leading underscores from the resulting string
    while (!new_string.empty() && new_string[0] == '_') {
        new_string.erase(0, 1);
    }

    // Step 5: Append ".txt" to the new string
    new_string += ".txt";

    return new_string;
}

void showProgressBar(size_t current, size_t total, size_t barWidth)
{
    float progress = static_cast<float>(current) / total;
    size_t pos = static_cast<size_t>(barWidth * progress);

    std::cout << "[";
    for (size_t i = 0; i < barWidth; ++i) {
        if (i < pos)
            std::cout << "=";
        else if (i == pos)
            std::cout << ">";
        else
            std::cout << " ";
    }
    std::cout << "] " << int(progress * 100.0) << " %\r";
    std::cout.flush();
}
