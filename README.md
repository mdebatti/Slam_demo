# Kalman Filter Vehicle Tracking

This repository contains a C++ implementation of a Kalman Filter applied to a vehicle tracking system. The project simulates a vehicle's movement along a predefined path, generates noisy sensor observations, and uses the Kalman Filter algorithm to estimate the vehicle's true position and orientation.

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

Given the current state vector and control input, the next state is predicted as:

![State Prediction Formula](https://latex.codecogs.com/png.image?%5Cmathbf%7Bx%7D_k%20%3D%20%5Cmathbf%7BF%7D_k%20%5Cmathbf%7Bx%7D_%7Bk-1%7D%20%2B%20%5Cmathbf%7BB%7D_k%20%5Cmathbf%7Bu%7D_k%20%2B%20%5Cmathbf%7Bw%7D_k)

Where:
- \( \mathbf{F}_k \) is the state transition matrix, representing how the state evolves from one time step to the next.
- \( \mathbf{B}_k \) is the control input matrix, relating the control input to the change in state.

### 2. Measurement Update

After the prediction, the Kalman Filter updates the state estimate using the measurement \( \mathbf{z}_k \). The measurement model is:

![Measurement Formula](https://latex.codecogs.com/png.image?%5Cmathbf%7Bz%7D_k%20%3D%20%5Cmathbf%7BH%7D_k%20%5Cmathbf%7Bx%7D_k%20%2B%20%5Cmathbf%7Bv%7D_k)

Where:
- \( \mathbf{H}_k \) is the observation matrix, relating the state to the measurement.
- \( \mathbf{v}_k \) is the measurement noise, assumed to be Gaussian with zero mean and covariance \( \mathbf{R}_k \).

The Kalman Gain \( \mathbf{K}_k \) is calculated as:

![Kalman Gain Formula](https://latex.codecogs.com/png.image?%5Cmathbf%7BK%7D_k%20%3D%20%5Cmathbf%7BP%7D_k%20%5Cmathbf%7BH%7D_k%5E%7BT%7D%20%5Cleft%28%20%5Cmathbf%7BH%7D_k%20%5Cmathbf%7BP%7D_k%20%5Cmathbf%7BH%7D_k%5E%7BT%7D%20%2B%20%5Cmathbf%7BR%7D_k%20%5Cright%29%5E%7B-1%7D)

The state estimate is then updated using the Kalman Gain:

![State Update Formula](https://latex.codecogs.com/png.image?%5Cmathbf%7Bx%7D_k%20%3D%20%5Cmathbf%7Bx%7D_k%20%2B%20%5Cmathbf%7BK%7D_k%20%​⬤
