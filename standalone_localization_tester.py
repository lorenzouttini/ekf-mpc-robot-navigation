#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt

from robot_localization_system import FilterConfiguration, Map, RobotEstimator

def wrap_angle(angle): return np.arctan2(np.sin(angle), np.cos(angle))

# Simulator configuration; this only contains
# stuff relevant for the standalone simulator.


class SimulatorConfiguration(object):
    def __init__(self):
        self.dt = 0.1
        self.total_time = 1000
        self.time_steps = int(self.total_time / self.dt)

        # Control inputs (linear and angular velocities)
        self.v_c = 1.0  # Linear velocity [m/s]
        self.omega_c = 0.1  # Angular velocity [rad/s]

# Placeholder for a controller.


class Controller(object):
    def __init__(self, config):
        self._config = config

    def next_control_input(self, x_est, Sigma_est):
        return [self._config.v_c, self._config.omega_c]

# This class implements a simple simulator for the unicycle
# robot seen in the lectures.


class Simulator(object):

    # Initialize
    def __init__(self, sim_config, filter_config, map):
        self._config = sim_config
        self._filter_config = filter_config
        self._map = map

    # Reset the simulator to the start conditions
    def start(self):
        self._time = 0
        self._x_true = np.random.multivariate_normal(self._filter_config.x0,
                                                     self._filter_config.Sigma0)
        self._u = [0, 0]

    def set_control_input(self, u):
        self._u = u

    # Predict the state forwards to the next timestep
    def step(self):
        dt = self._config.dt
        v_c = self._u[0]
        omega_c = self._u[1]
        v = np.random.multivariate_normal(
            mean=[0, 0, 0], cov=self._filter_config.V * dt)
        self._x_true = self._x_true + np.array([
            v_c * np.cos(self._x_true[2]) * dt,
            v_c * np.sin(self._x_true[2]) * dt,
            omega_c * dt
        ]) + v
        self._x_true[-1] = np.arctan2(np.sin(self._x_true[-1]),
                                      np.cos(self._x_true[-1]))
        self._time += dt
        return self._time

    # Get the observations to the landmarks. Return None if none visible
    def landmark_range_observations(self):
        y = []
        C = []
        W = self._filter_config.W_range
        for lm in self._map.landmarks:
            # True range measurement (with noise)
            dx = lm[0] - self._x_true[0]
            dy = lm[1] - self._x_true[1]
            range_true = np.sqrt(dx**2 + dy**2)
            range_meas = range_true + np.random.normal(0, np.sqrt(W))
            y.append(range_meas)

        y = np.array(y)
        return y
    
    # Get the range + bearings observations to the landmarks. Return None if none visible
    def landmark_range_bearing_observations(self):
        # y_range = []
        # y_bearing = []
        y = []
        C = []
        W_range = self._filter_config.W_range
        W_bearing = self._filter_config.W_bearing
        for lm in self._map.landmarks:
            # True range measurement (with noise)
            dx = lm[0] - self._x_true[0]
            dy = lm[1] - self._x_true[1]
            range_true = np.sqrt(dx**2 + dy**2)
            bearing_true = np.arctan2(dy, dx) - self._x_true[2]
            range_meas = range_true + np.random.normal(0, np.sqrt(W_range))
            bearing_meas = bearing_true + np.random.normal(0, np.sqrt(W_bearing))
            bearing_meas = wrap_angle(bearing_meas)
            # y_range.append(range_meas)
            # y_bearing.append(wrap_angle(bearing_meas))
            y.append([range_meas, bearing_meas])

        # y_range = np.array(y_range)
        # y_bearing = np.array(y_bearing)
        # return y_range, y_bearing
        y = np.array(y)
        return y
    
    def x_true(self):
        return self._x_true


# Create the simulator configuration.
sim_config = SimulatorConfiguration()

# Create the filter configuration. If you want
# to investigate mis-tuning the filter,
# create a different filter configuration for
# the simulator and for the filter, and
# change the parameters between them.
filter_config = FilterConfiguration()

# Create the map object for the landmarks.
map = Map()
# print(map.landmarks)
# map.generate_square_grid(grid_size=12, spacing=2)
map.generate_square_boundary(side_length=40, num_landmarks_per_side=11)
# print(map.landmarks)
# map.generate_circular_boundary(radius=20, num_landmarks=50)
# map.generate_circular_grid(center=(0,0), radius=11, spacing=2)
# map.generate_triangular_boundary(num_landmarks=40, center=(0,0), side_length=40)
# map.generate_triangular_grid(center=(0, 0), spacing=2, num_rows=15)

num_landmarks = map.num_landmarks
landmark_type = map.landmark_type
print(f"Landmark type: {landmark_type}. Number of Landmarks: {num_landmarks}")

# Create the controller. This just provides
# fixed control inputs for now.
controller = Controller(sim_config)

# Create the simulator object and start it.
simulator = Simulator(sim_config, filter_config, map)
simulator.start()

# Create the estimator and start it.
estimator = RobotEstimator(filter_config, map)
estimator.start()

# Extract the initial estimates from the filter
# (which are the initial conditions) and use
# these to generate the control for the first timestep.
x_est, Sigma_est = estimator.estimate()
u = controller.next_control_input(x_est, Sigma_est)

# Arrays to store data for plotting
x_true_history = []
x_est_history = []
Sigma_est_history = []

# Main loop
for step in range(sim_config.time_steps):

    # Set the control input and propagate the
    # step the simulator with that control iput.
    simulator.set_control_input(u)
    simulation_time = simulator.step()

    # Predict the Kalman filter with the same
    # control inputs to the same time.
    estimator.set_control_input(u)
    estimator.predict_to(simulation_time)
    # print(f"\n SimTime is: {simulation_time} \n")
    
    # # Get the landmark observations.
    # y = simulator.landmark_range_observations()
    # # Update the filter with the latest observations.
    # estimator.update_from_landmark_range_observations(y)

    y = simulator.landmark_range_bearing_observations()
    estimator.update_from_landmark_range_bearing_observations(y)

    # Get the current state estimate.
    x_est, Sigma_est = estimator.estimate()

    # Figure out what the controller should do next.
    u = controller.next_control_input(x_est, Sigma_est)

    # Store data for plotting.
    x_true_history.append(simulator.x_true())
    x_est_history.append(x_est)
    Sigma_est_history.append(np.diagonal(Sigma_est))

# Convert history lists to arrays.
x_true_history = np.array(x_true_history)
x_est_history = np.array(x_est_history)
Sigma_est_history = np.array(Sigma_est_history)

# Plotting the true path, estimated path, and landmarks.
plt.figure(figsize=(8,6))
plt.plot(x_true_history[:, 0], x_true_history[:, 1], label='True Path')
plt.plot(x_est_history[:, 0], x_est_history[:, 1], label='Estimated Path')
plt.scatter(map.landmarks[:, 0], map.landmarks[:, 1],
            marker='x', color='red', label='Landmarks')
plt.legend()
plt.xlabel('X position [m]')
plt.ylabel('Y position [m]')
plt.title(f'Unicycle Robot Localization using EKF with {num_landmarks} landmarks')
plt.axis('equal')
plt.grid(True)
plt.savefig(f'figures/localization_{landmark_type}_{num_landmarks}.png')
plt.show()

# Note the angle state theta experiences "angles
# wrapping". This small helper function is used
# to address the issue.


def wrap_angle(angle): return np.arctan2(np.sin(angle), np.cos(angle))


# Plot the 2 standard deviation and error history for each state.
state_name = ['x', 'y', 'θ']
estimation_error = x_est_history - x_true_history
estimation_error[:, -1] = wrap_angle(estimation_error[:, -1])
threshold = 0.1
for s in range(3):
    plt.figure(figsize=(8,6))
    two_sigma = 2*np.sqrt(Sigma_est_history[:, s])
    plt.plot(estimation_error[:, s], label="Estimation Error")
    plt.plot(two_sigma, linestyle='dashed', color='red', label="2σ bound")
    plt.plot(-two_sigma, linestyle='dashed', color='red')

    if s < 2:  # Only apply to x and y as they are position measurements
        plt.axhline(threshold, color='green', linestyle=':', label="10 cm threshold")
        plt.axhline(-threshold, color='green', linestyle=':')
    
        # Calculate mean absolute error
        mean_abs_error = np.mean(np.abs(estimation_error[:, s]))

        # Display the mean absolute error on the plot
        plt.plot([], [], label=f'Mean Abs Est. Error: {mean_abs_error*100:.2f} cm', color='black', marker='.', linestyle='')

        # # Display the mean absolute error on the plot
        # plt.text(0.5, 0.9, f'Mean Abs Error: {mean_abs_error:.4f} m', 
        #         transform=plt.gca().transAxes, fontsize=10, 
        #         verticalalignment='top', horizontalalignment='center')
    plt.title(f"{state_name[s]} estimation error and 2σ bounds")
    plt.xlabel("Timestep")
    plt.ylabel(f"Error in {state_name[s]}")
    plt.legend()
    plt.savefig(f'figures/error_{state_name[s]}_{landmark_type}_{num_landmarks}.png')
    plt.show()

    # plt.title(state_name[s])
    # plt.legend()
    # plt.show()
