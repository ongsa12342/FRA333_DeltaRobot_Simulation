import numpy as np
import matplotlib.pyplot as plt

def trapezoidal_trajectory(initial_position, target_position, max_velocity, max_acceleration):
    # Calculated Distance
    s = abs(target_position - initial_position)  
    # Define Direction 
    direction = np.sign(target_position - initial_position) 

    # Define pattern of trapezoidal_trajectory
    if s > (max_velocity ** 2) / max_acceleration:
        # Calculate the time required for acceleration and deceleration for
        time_accel = max_velocity / max_acceleration
        time_total = 2 * time_accel + (s - (max_velocity ** 2) / max_acceleration) / max_velocity
    else:
        # Calculate the time required for acceleration and deceleration
        time_accel = np.sqrt(s / max_acceleration)
        time_total = 2 * time_accel

    # Generate the time array
    time = np.arange(0, time_total, 0.01)

    # Initialize the position, velocity, and acceleration arrays
    position = np.zeros_like(time)
    velocity = np.zeros_like(time)
    acceleration = np.zeros_like(time)

    # Calculate the position, velocity, and acceleration for each time step
    for i, t in enumerate(time):
        if t <= time_accel:
            # Acceleration phase
            velocity[i] = max_acceleration * t * direction
            position[i] = initial_position + direction * 0.5 * max_acceleration * t ** 2
            acceleration[i] = max_acceleration * direction
            max_velocity = velocity[i]
        elif t < time_total - time_accel:
            # Constant velocity phase
            velocity[i] = max_velocity
            position[i] = (
                initial_position
                + 0.5 * direction * max_acceleration * time_accel ** 2
                + max_velocity * (t - time_accel)
            )
            acceleration[i] = 0
        elif t >= time_total - time_accel:
            # Deceleration phase
            velocity[i] = max_velocity - max_acceleration * direction * (t - (time_total - time_accel))
            position[i] = (
                initial_position
                + 0.5 * direction * max_acceleration * time_accel ** 2
                + max_velocity * (time_total - 2 * time_accel)
                - 0.5 * direction * max_acceleration * (t - (time_total - time_accel)) ** 2
                + max_velocity * (t - (time_total - time_accel))
            )
            acceleration[i] = -max_acceleration * direction

    # Plot results
    plt.subplot(3, 1, 1)
    plt.plot(time, position)
    plt.title('Trapezoidal trajectory')
    plt.xlabel('time (sec)')
    plt.ylabel('position (mm)')

    plt.subplot(3, 1, 2)
    plt.plot(time, velocity)
    plt.xlabel('time (sec)')
    plt.ylabel('velo (mm/s)')

    plt.subplot(3, 1, 3)
    plt.plot(time, acceleration)
    plt.xlabel('time (sec)')
    plt.ylabel('accel (mm/s^2)')

    plt.show()

# Example usage:
# (initial_position, target_position, max_velocity, max_acceleration)
trapezoidal_trajectory(0, 10, 50, 50)
