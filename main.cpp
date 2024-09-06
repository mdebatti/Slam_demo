#include "constants.h"
#include "KalmanFilter.h"
#include "SimulationSetup.h"

#include <Eigen/Dense>

int main(void)
{
    Logger logger("slam_simulation_output.txt");

    // To later save in a text file
    DataMatrix x_vehicle_pred, x_vehicle_est, u_noisy, innovation, innovation_cov;
    DataVector chi2;
    try
    {
        // Load the data from file - see "constants.h" for details
        SimulationSetup sim_data(FILE_REFPATH, FILE_BEACONS, SLAM_CONST::NUM_LOOPS);

        // Initialise Kalman filter with initial vehicle position
        // from the input txt file, all noise parameters in constants.h
        KalmanFilter aEKF(logger,sim_data.getInitialVehicleState());

        std::cout << "Running the EKF Slam demo:" << endl;

        logger.log(-1, "Running the EKF Slam demo");

        for(int k = 0; k < sim_data.get_num_steps(); ++k)
        {

            // Prediction step
            Kalman::Input u = sim_data.getControlInputsNoisy(k);
            double n = sim_data.getWheelRadiusNoise(k);
            aEKF.predictState(u, n);

            // Log to txt
            aEKF.logControlInput(k, u);
            aEKF.logStateAndCovariance(k, "State and Covariance after prediction");

            // Log state & noisy inputs for plotting
            x_vehicle_pred.push_back(aEKF.getVehicleStates());
            u_noisy.push_back(DataVector(u.data(), u.data() + u.size()));

            // Update step
            Kalman::ObservationWithTag z = sim_data.getNoisyObservationWithTag(k);

            //log observation
            aEKF.logObservation(k, z);

            if(aEKF.isMapped(z))
            {
                aEKF.updateEKF(z);
            }
            else
            {
                aEKF.addState(z);
            }
            aEKF.logStateAndCovariance(k, "State and Covariance after update");
            aEKF.logInnovationAndCovariance(k, z, "Innovation vector and its covariance after update");

            // Save state
            x_vehicle_est.push_back(aEKF.getVehicleStates());
            innovation.push_back(aEKF.getInnovation());
            innovation_cov.push_back(aEKF.getInnovationCov(z));
            chi2.push_back(aEKF.get_chi2());

            // Update the progress bar
            showProgressBar(k, sim_data.get_num_steps());
        }
    }
    catch(const std::exception &ex)
    {
        std::cout << ex.what() << endl;
        return EXIT_FAILURE;
    }

    saveDataToFile(x_vehicle_pred, generateNewFilename("x_vehicle_pred", FILE_REFPATH));
    saveDataToFile(x_vehicle_est, generateNewFilename("x_vehicle_est", FILE_REFPATH));
    saveDataToFile(u_noisy, generateNewFilename("u_noisy", FILE_REFPATH));
    saveDataToFile(innovation, generateNewFilename("innovation", FILE_REFPATH));
    saveDataToFile(innovation_cov, generateNewFilename("innovationCovariance", FILE_REFPATH));
    saveDataToFile(chi2, generateNewFilename("chi2_innovation_gate", FILE_REFPATH));

    std::cout << "Finished simulation without errors" << endl;
    return(EXIT_SUCCESS);
}

