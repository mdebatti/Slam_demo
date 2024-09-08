# Kalman Filter Vehicle Tracking

This repository contains a C++ implementation of a Kalman Filter applied to a vehicle tracking system. The project simulates a vehicle's movement along a predefined path, generates noisy sensor observations, and uses the Kalman Filter algorithm to estimate the vehicle's true position and orientation.

[![Alt Text](https://github.com/mdebatti/Slam_demo/blob/main/video/EKF_SLAM.gif)](https://github.com/mdebatti/Slam_demo/blob/main/video/EKF_SLAM.mp4)

- Red dot: vehicle
- blue path:            vehicle trajectory
- red path:             estimated trajectory
- Green circles:        landmarks
- Green dashed circles: observation range (landmark visible to vehicle radar)

note how the estimated path moves away from the true path when landmarks are not observable, and how the estimated path snaps back to the true trajectory as soon as a previously observed feature is re-observed.

The C++ implementation in this repository compiles and run. The predict(), update(), addState() member functions are in agreement with the output of the original matlab implementation shown on the animation. There is however a bug that causes the covariance matrixes and the estimate to diverge in the C++ code.

## Features

- **Control Input Generation**: Calculate ideal control inputs (e.g., steering, velocity) based on a target path to predict the vehicle's true state.
- **Observation Simulation**: Generate simulated sensor data with added noise to mimic real-world sensor imperfections.
- **Kalman Filter Implementation**: Iteratively apply the Kalman Filter algorithm to predict the vehicle's state, incorporate noisy observations, and refine the state estimate.

## How It Works

### 1. Control Input Generation

Given a target path, the system calculates the necessary control inputs to follow the path. These inputs are used in a kinematic or dynamic model to compute the vehicle's true position and orientation, assuming ideal conditions without noise.

### 2. Simulated Observations

Simulated sensor readings are generated based on the vehicle's true position and orientation. Gaussian noise is added to these readings to create more realistic observations that the Kalman Filter will later process.

### 3. Kalman Filter Process

The Kalman Filter operates in a cycle of three main steps:

- **Prediction**: 
  - Predict the next state of the vehicle using the current state estimate and control input.
  - Apply matrix operations to update the state estimate and covariance matrix.

- **Measurement Update**:
  - Use the noisy observations to refine the state estimate.
  - Calculate the Kalman Gain to optimally combine the prediction and observation.

- **Map Expansion** (if applicable):
  - Expand the state vector and covariance matrix if new features or landmarks are detected.

### 4. Noise Modeling

- **Process Noise**: Reflects uncertainty in the vehicle's motion model, added during the prediction step.
- **Measurement Noise**: Reflects sensor inaccuracies, added to the simulated observations.

## Detailed Matrix Operations

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

- A C++ compiler supporting C++11 or later.
- CMake for building the project.
- Eigen for the matrix operations

### Building the Project

1. Clone the repository:

    ```bash
    git clone https://github.com/mdebatti/Slam_demo.git
    cd Slam_demo
    ```

2. Build the project using CMake:

    ```bash
    mkdir build
    cd build
    cmake ..
    make
    ```

3. Run the executable:

    ```bash
    ./KalmanFilter
    ```

### File Descriptions

- **main.cpp**: Entry point of the application; sets up the simulation and runs the Kalman Filter.
- **KalmanFilter.cpp**: Implements the Kalman Filter algorithm.
- **KalmanFilter.h**: Header file containing the Kalman Filter class definition.
- **SimulationSetup.cpp**: Contains setup and configuration for the simulation environment, including initializing the target path and generating control inputs.
- **SimulationSetup.h**: Header file for the simulation setup module.
- **types.h**: Defines types and constants used throughout the simulation.

## Contributing

Contributions are welcome! Please submit a pull request or open an issue to discuss your ideas.

**Reference:**

[1] Hugh F. Durrant-Whyte, *An Autonomous Guided Vehicle for Cargo Handling Applications*, The International Journal of Robotics Research, Vol. 15, No. 5, October 1996, pp. 407-440.
[Download PDF](./docs/durrant-whyte1996.pdf)

[2] S. Clark, H. Durrant-Whyte, *Autonomous Land Vehicle Navigation Using Millimeter Wave Radar*, Proceedings of the 1998 IEEE International Conference on Robotics & Automation, Leuven, Belgium, May 1998, Australian Centre for Field Robotics, Sydney University, Australia.
[Download PDF](./docs/autonomous-land-vehicle-navigation-using-millimeter-wave-radar.pdf)
