# Extended Kalman Filter SLAM Implementation in C++

## Overview


This repository contains an implementation of the Extended Kalman Filter (EKF) for Simultaneous Localization and Mapping (SLAM) using modern C++. The project simulates a vehicle's movement along a predefined path, generates noisy sensor observations, and uses the Kalman Filter algorithm to estimate the vehicle's true position and orientation. Navigational features are incorporated into the system state when they are first observed.

![Extended Kalman Filter SLAM Implementation in C++](https://github.com/mdebatti/Slam_demo/blob/main/video/EKF_SLAM.gif)

- Red dot: vehicle
- blue path:            vehicle trajectory
- red path:             estimated trajectory
- Green circles:        landmarks
- Green dashed circles: observation range (landmark visible to vehicle radar)

Notice how the estimated path deviates from the true path when landmarks are not observable due to the influence of process noise on the prediction model. However, as soon as a previously observed landmark is detected again, the estimated path quickly realigns with the true trajectory.

The figure below illustrates the innovations (differences between the observed and predicted measurements) for both "range" and "bearing" observations in the Extended Kalman Filter SLAM implementation. The innovations are plotted along with the 2-sigma (95% confidence) bounds derived from the innovation covariance matrix. Throughout the simulation, the majority of innovation values consistently fall within the 2-sigma bounds, confirming that the filter's predictions are well-calibrated and that the measurement noise is accurately modeled.

![Innovation in C++](https://github.com/mdebatti/Slam_demo/blob/main/figures/innovation_C++.png)

## Features
- **Extended Kalman Filter Implementation**: EKF for SLAM applications.
- **Control Input Generation**: Calculate ideal control inputs (e.g., steering, velocity) based on a target path to predict the vehicle's true state.
- **Observation Simulation**: Generate simulated radar observations with added noise to mimic real-world sensor imperfections.
- **Efficient Matrix Operations**: Utilizes the Eigen library for high-performance linear algebra computations.
- **Modern C++ Design Patterns:** Employs the Strategy and Decorator patterns for flexibility and modularity.
- **Self-testing Kalman Filter:** The Kalman Filter object performs self-tests on its matrix operations upon construction to ensure correctness.

## How It Works

### 1. Control Input Generation

Given a target path, the system calculates the necessary control inputs to follow the path. These inputs are used in a kinematic model to compute the vehicle's true position and orientation, assuming ideal conditions without noise.

### 2. Simulated Observations

Simulated sensor readings are generated based on the vehicle's true position and orientation. Gaussian noise is added to these readings to create more realistic observations that the Kalman Filter will later process.

### 3. Kalman Filter Process

The Kalman Filter operates in a cycle of two main steps:

- **Prediction**: 
  - Predict the next state of the vehicle using the current state estimate and control input.
  - Apply matrix operations to update the state estimate and covariance matrix.

- **Measurement Update & Map Expansion (if applicable)**:
  - Use the noisy observations to refine the state estimate.
  - Calculate the Kalman Gain to optimally combine the prediction and observation.
  - Expand the state vector and covariance matrix if new features or landmarks are detected.
 

  ```cpp
  Kalman::ObservationWithTag z = sim_data.getNoisyObservationWithTag(k);

  if(loggingKalmanFilter->isMapped(z))
  {
      loggingKalmanFilter->update();
  }
  else
  {
      loggingKalmanFilter->addState();
  }
  ```

**Self-testing Matrix Operations**: Upon construction, the Kalman Filter object performs self-tests on its matrix operations to ensure all mathematical computations are correctly implemented.

### 4. Noise Modeling

- **Process Noise**: Reflects uncertainty in the vehicle's motion model, added during the prediction step.
- **Measurement Noise**: Reflects sensor inaccuracies, added to the simulated observations.

## Detailed Matrix Operations

### Matrix Operations

Matrix computations are central to the EKF algorithm. This project leverages the [Eigen](https://eigen.tuxfamily.org/) library for efficient and reliable linear algebra operations, including:

- Matrix multiplication and inversion
- Calculation of Jacobians for linearization
- Covariance updates
- State estimation updates

### 1. State Prediction

Given the current state vector `x(k-1)` and control input `u(k)`, the next state `x(k)` is predicted as:

x(k) = F(k) * x(k-1) + B(k) * u(k) + w(k)

Where:
- `F(k)` is the state transition matrix, representing how the state evolves from one time step to the next.
- `B(k)` is the control input matrix, relating the control input `u(k)` to the change in state.
- `w(k)` is the process noise, which accounts for uncertainty in the prediction.

The covariance matrix `P(k)` is updated as:

P(k) = F(k) * P(k-1) * F(k)’ + Q(k)

Where `Q(k)` is the process noise covariance matrix.

### 2. Measurement Update

After the prediction, the Kalman Filter updates the state estimate using the measurement `z(k)`. The measurement model is:

z(k) = H(k) * x(k) + v(k)

Where:
- `H(k)` is the observation matrix, relating the state to the measurement.
- `v(k)` is the measurement noise, with covariance `R(k)`.

The Kalman Gain `K(k)` is calculated as:

K(k) = P(k) * H(k)’ * inv(H(k) * P(k) * H(k)’ + R(k))

The state estimate is then updated using the Kalman Gain:

x(k) = x(k) + K(k) * (z(k) - H(k) * x(k))

The covariance matrix is updated as:

P(k) = (I - K(k) * H(k)) * P(k)

Where `I` is the identity matrix.

### 3. Noise Handling

- **Process Noise (`w(k)`)**: Added during the prediction step to account for uncertainties in the system's dynamics.
- **Measurement Noise (`v(k)`)**: Added to the measurements to account for sensor inaccuracies.

## Getting Started

### Prerequisites

- A C++ compiler supporting C++17 or later.
- CMake for building the project.
- Eigen for the matrix operations (included in this repository)

### Building the Project

1. Clone the repository:

    ```bash
    git clone https://github.com/mdebatti/Slam_demo.git
    cd Slam_demo
    ```

2. Build the project:

3. Run the executable:


## Repository Structure

- `thirdparty/`: Eigen library
- `docs/`: references
- `video/`: animation illustrating a vehicle tracking application
- `figures/`: plots showing agreement with Matlab code
- `inputs/`: location of landmarks and vehicle reference path in txt files, and Matlab results
- `data/`: Input data files for simulation
- `outputs/`: folder created upon execution. logs as .txt file. Some can me directly loaded into matlab as variables.
- `CMakeLists.txt`: Build configuration

## Dependencies

- **Eigen Library**: Used for all matrix and linear algebra operations. [Eigen Website](https://eigen.tuxfamily.org/)
- **Standard C++ Libraries**

## Project Structure

The project is organized into several key files and directories that encapsulate different aspects of the Extended Kalman Filter SLAM implementation. Below is a breakdown of the major components:

### Header Files

- **`constants.h`**: Contains physical constants, noise parameters, and configuration settings for the SLAM simulation.
- **`FileIO.h`**: Defines the `FileIO` class for handling file input/output operations, particularly reading simulation data and saving results.
- **`functions.h`**: Includes utility functions used throughout the project, such as angle normalization and file name generation.
- **`KalmanFilter.h`**: The core implementation of the Kalman Filter, including prediction, update, and state management functions.
- **`KalmanFilterStrategy.h`**: Defines an abstract base class (`KalmanFilterStrategy`) for implementing different Kalman Filter strategies, facilitating the use of the Strategy design pattern.
- **`Logger.h`**: Declares the `Logger` class responsible for logging simulation data and Kalman Filter states for later analysis.
- **`LoggingKalmanFilterDecorator.h`**: Implements the Decorator design pattern, allowing the addition of logging functionality to the Kalman Filter without altering its core behavior.
- **`SimulationSetup.h`**: Contains the `SimulationSetup` class, which initializes the simulation environment, including vehicle state and sensor noise.
- **`types.h`**: Defines various type aliases and data structures used throughout the project, such as matrices and vectors for linear algebra operations.

### Source Files

- **`FileIO.cpp`**: Implements the `FileIO` class methods for reading simulation input data and writing output data to files.
- **`functions.cpp`**: Implements various utility functions declared in `functions.h`.
- **`KalmanFilter.cpp`**: Contains the implementation of the Kalman Filter, including matrix operations for the prediction and update steps.
- **`KalmanFilterStrategy.cpp`**: The source file corresponding to the `KalmanFilterStrategy` class, providing an interface for different Kalman Filter implementations.
- **`Logger.cpp`**: Implements the `Logger` class, which logs the Kalman Filter’s internal state and simulation results.
- **`LoggingKalmanFilterDecorator.cpp`**: Implements the methods of the `LoggingKalmanFilterDecorator` class, adding logging capabilities to the Kalman Filter.
- **`main.cpp`**: The entry point of the application, where the simulation is set up, the Kalman Filter is instantiated, and the main loop is executed.
- **`SimulationSetup.cpp`**: Implements the `SimulationSetup` class, which initializes and manages the simulation environment, including control inputs and sensor observations.

### CMake Configuration

- **`CMakeLists.txt`**: The CMake configuration file for building the project, specifying the source files, include directories, and necessary libraries.

## Acknowledgments
Thank you to Prof. Hugh F. Durrant-Whyte and Prof. Stefan B. Williams for their support and for allowing me to complete my master’s thesis at the Australian Centre for Field Robotics at the University of Sydney. This work draws on many ideas developed at the center, and their guidance was invaluable.

## Reference:

[1] Hugh F. Durrant-Whyte, *An Autonomous Guided Vehicle for Cargo Handling Applications*, The International Journal of Robotics Research, Vol. 15, No. 5, October 1996, pp. 407-440.
[Download PDF](./docs/durrant-whyte1996.pdf)

[2] S. Clark, H. Durrant-Whyte, *Autonomous Land Vehicle Navigation Using Millimeter Wave Radar*, Proceedings of the 1998 IEEE International Conference on Robotics & Automation, Leuven, Belgium, May 1998, Australian Centre for Field Robotics, Sydney University, Australia.
[Download PDF](./docs/autonomous-land-vehicle-navigation-using-millimeter-wave-radar.pdf)
