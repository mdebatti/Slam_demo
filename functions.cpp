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
    while (!new_string.empty() && new_string[0] == '_')
    {
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
    for (size_t i = 0; i < barWidth; ++i)
    {
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

template <typename T>
void saveDataToFile(const T& data, const std::string& filename)
{
    // Define the relative path to the outputs directory
     std::string outputDir = FILE_OUTPUT_DIR;

     // Create the directory if it doesn't exist
     std::filesystem::create_directories(outputDir);

     // Append the filename to the output directory path
     std::string fullPath = outputDir + filename;

     std::ofstream file(fullPath);
     if (!file.is_open())
     {
         std::cerr << "Failed to open file: " << fullPath << std::endl;
         return;
     }

    file.precision(20);

    // Handle 2D vector (DataMatrix)
    if constexpr (std::is_same_v<T, DataMatrix>)
    {
        for(int rr = 0; rr < data.size(); ++rr )
        {
            int n_cols = data[rr].size();
            for(int cc = 0; cc < n_cols; ++cc)
            {
                file << data[rr][cc];
                if (cc < n_cols - 1)  // Only add a space if it's not the last element
                {
                    file << " ";
                }
            }
            file << '\n';  // Move to the next line after all columns are written
        }
    }

    // Handle 1D vector (DataVector)
    if constexpr (std::is_same_v<T, DataVector>)
    {
        for(int rr = 0; rr < data.size(); ++rr )
        {
            file << data[rr] << '\n';
        }
    }

    file.close();
}

template <typename Matrix, typename Matrix2>
void testEigenMaxtrixEquality(const Matrix& M1, const Matrix2& M2, const std::string& label_str, const std::string& label_str2)
{
    if (M1.isApprox(M2))
    {
        std::cout << "\nMatrix multiplication test for " << label_str2 << " step passed (" << label_str << ")!\n" << std::endl;
    }
    else
    {
        std::ostringstream msg;
        msg << "Matrix manipulation results are wrong in " << label_str2 << " in " << __func__ << "(" << __FILE__ << ")\n" << std::endl;
        std::cerr << msg.str();

        std::cout << label_str << ":\n" << M1 << "\n\n";
        std::cout << label_str << " (expected output):\n\n" << M2 << std::endl;

        throw std::runtime_error(msg.str());
    }
}

// Explicit template instantiation
template void testEigenMaxtrixEquality<Kalman::ObservationCovariance, Kalman::ObservationCovariance>(const Kalman::ObservationCovariance& M1, const Kalman::ObservationCovariance& M2, const std::string& label_str, const std::string& label_str2);
template void testEigenMaxtrixEquality<Kalman::KalmanGain, Kalman::KalmanGain>(const Kalman::KalmanGain& M1, const Kalman::KalmanGain& M2, const std::string& label_str, const std::string& label_str2);
template void testEigenMaxtrixEquality<Kalman::State, Kalman::State>(const Kalman::State& M1, const Kalman::State& M2, const std::string& label_str, const std::string& label_str2);
template void testEigenMaxtrixEquality<Kalman::StateCovariance, Kalman::StateCovariance>(const Kalman::StateCovariance& M1, const Kalman::StateCovariance& M2, const std::string& label_str, const std::string& label_str2);

// Explicit template instantiation
template void saveDataToFile<DataMatrix>(const DataMatrix& data, const std::string& filename);
template void saveDataToFile<DataVector>(const DataVector& data, const std::string& filename);



