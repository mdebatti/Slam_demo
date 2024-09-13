#include "constants.h"
#include "KalmanFilter.h"
#include "LoggingKalmanFilterDecorator.h"
#include "SimulationSetup.h"
#include <memory>
#include <Eigen/Dense>

int main(void)
{
    // To later save in a text file
    DataMatrix  x_vehicle_pred, x_vehicle_est, u_noisy,
                innovation, innovation_cov, z_obs, z_pred,
                x_vehicle_stdev;
    DataVector chi2;

    try
    {
        // Load the data from file - see "constants.h" for details
        SimulationSetup sim_data(FILE_REFPATH, FILE_BEACONS, SLAM_CONST::NUM_LOOPS);

        //Setup logger
        Logger logger;

        // Create the KalmanFilter object
        std::unique_ptr<KalmanFilterStrategy> kalmanFilter = std::make_unique<KalmanFilter>(logger, sim_data.getInitialVehicleState());

        // Wrap it with the LoggingKalmanFilterDecorator
        auto loggingKalmanFilter = std::make_unique<LoggingKalmanFilterDecorator>(std::move(kalmanFilter), logger);

        std::cout << "Running the EKF Slam demo:" << endl;


        for(int k = 0; k < 1000; ++k) //sim_data.get_num_steps()
        {
            // Prediction step
            Kalman::Input u = sim_data.getControlInputsNoisy(k);
            double n = sim_data.getWheelRadiusNoise(k);
            loggingKalmanFilter->predict(u,n);

            // Update step
            Kalman::ObservationWithTag z = sim_data.getNoisyObservationWithTag(k);

            if(loggingKalmanFilter->isMapped(z))
            {
                loggingKalmanFilter->update();
            }
            else
            {
                loggingKalmanFilter->addState();
            }
            showProgressBar(k, sim_data.get_num_steps());
        }

        // Save all logged data to files
        logger.saveAllData();
    }
    catch(const std::exception &ex)
    {
        std::cout << ex.what() << endl;
        return EXIT_FAILURE;
    }

    std::cout << "Finished simulation without errors" << endl;
    return(EXIT_SUCCESS);
}

