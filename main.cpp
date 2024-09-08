#include "constants.h"
#include "KalmanFilter.h"
#include "SimulationSetup.h"
#include <memory>
#include <Eigen/Dense>

int main(void)
{
    // To later save in a text file
    DataMatrix x_vehicle_pred, x_vehicle_est, u_noisy, innovation, innovation_cov, z_obs;
    DataVector chi2;

    try
    {
        // Log simulation outputs in  ~\src\Slam\cpp\outputs
        Logger logger("slam_simulation_output.txt");

        // Load the data from file - see "constants.h" for details
        SimulationSetup sim_data(FILE_REFPATH, FILE_BEACONS, SLAM_CONST::NUM_LOOPS);

        // Construct the KalmanFilter and store it in a unique_ptr of type KalmanFilterStrategy
        std::unique_ptr<KalmanFilterStrategy> filter = std::make_unique<KalmanFilter>(logger, sim_data.getInitialVehicleState());

        std::cout << "Running the EKF Slam demo:" << endl;

        for(int k = 0; k < sim_data.get_num_steps(); ++k)
        {
            // Prediction step
            Kalman::Input u = sim_data.getControlInputsNoisy(k);
            double n = sim_data.getWheelRadiusNoise(k);
            filter->predict(u,n);

            // Update step
            Kalman::ObservationWithTag z = sim_data.getNoisyObservationWithTag(k);

            if(filter->isMapped(z))
            {
                filter->update(z);
            }
            else
            {
                filter->addState(z);
            }

            auto* kalman_filter = dynamic_cast<KalmanFilter*>(filter.get());
            if (kalman_filter)
            {
                // Log to a single text file
                kalman_filter->logControlInput(k, u);
                kalman_filter->logObservation(k, z);
                kalman_filter->logStateAndCovariance(k, "State and Covariance after update");
                kalman_filter->logInnovationAndCovariance(k, "Innovation vector and its covariance after update");
                kalman_filter->logStateToInnovationTransferMatrixH(k, "State Covariance to Innovation covariance transfer matrix H");

                // Log to separate text files
                x_vehicle_pred.push_back(kalman_filter->getVehicleStates());
                u_noisy.push_back(DataVector(u.data(), u.data() + u.size()));
                x_vehicle_est.push_back(kalman_filter->getVehicleStates());
                innovation.push_back(kalman_filter->getInnovation());
                innovation_cov.push_back(kalman_filter->getInnovationCov(z));
                z_obs.push_back(kalman_filter->getMeasurement());
                chi2.push_back(kalman_filter->get_chi2());
            }

            showProgressBar(k, sim_data.get_num_steps());
        }

        // Save to separate text files in /output folder
        saveDataToFile(x_vehicle_pred, generateNewFilename("x_vehicle_pred", FILE_REFPATH));
        saveDataToFile(x_vehicle_est, generateNewFilename("x_vehicle_est", FILE_REFPATH));
        saveDataToFile(u_noisy, generateNewFilename("u_noisy", FILE_REFPATH));
        saveDataToFile(innovation, generateNewFilename("innovation", FILE_REFPATH));
        saveDataToFile(innovation_cov, generateNewFilename("innovationCovariance", FILE_REFPATH));
        saveDataToFile(chi2, generateNewFilename("chi2_innovation_gate", FILE_REFPATH));
        saveDataToFile(z_obs, generateNewFilename("z_obs", FILE_REFPATH));
    }
    catch(const std::exception &ex)
    {
        std::cout << ex.what() << endl;
        return EXIT_FAILURE;
    }

    std::cout << "Finished simulation without errors" << endl;
    return(EXIT_SUCCESS);
}

