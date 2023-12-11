import numpy as np
import matplotlib.pyplot as plt

def trajectory_trapezoidal(initial_posJ, target_posJ, max_veloJ, max_accelJ, dt, current_time, end_time):
    # Check for division by zero
    if max_accelJ == 0:
        raise ValueError("max_accelJ must be non-zero.")

    # Check for negative time
    if current_time < 0:
        raise ValueError("current_time must be non-negative.")

    # Calculated Distance
    s = target_posJ - initial_posJ
    direction = np.sign(s)
    s = abs(s)
    
    # Define pattern of trapezoidal_trajectory
    if s > (max_veloJ ** 2) / max_accelJ:
        # Calculate the time required for acceleration and deceleration
        time_accel = max_veloJ / max_accelJ
        time_total = 2 * time_accel + (s - (max_veloJ ** 2) / max_accelJ) / max_veloJ
    else:
        # Calculate the time required for acceleration and deceleration
        time_accel = np.sqrt(s / max_accelJ)
        time_total = 2 * time_accel

    # Optional: Numeric stability check
    if np.isclose(current_time, time_accel):
        current_time = time_accel

    if current_time <= time_accel:
        # Acceleration phase
        velocity = max_accelJ * current_time * direction
        position = initial_posJ + direction * 0.5 * max_accelJ * current_time ** 2
        acceleration = max_accelJ * direction
    elif current_time < time_total - time_accel:
        # Constant velocity phase
        velocity = max_veloJ * direction
        position = (
                initial_posJ +
                direction *( 0.5  * max_accelJ * time_accel ** 2
                + max_veloJ * (current_time - time_accel))
        )
        acceleration = 0
        # print(current_time, position, velocity, acceleration)
    elif time_total - time_accel <= current_time < end_time:
        # Deceleration phase
        # print("Deceleration phase")
        decel_time = current_time - (time_total - time_accel)
        velocity = direction*(max_veloJ - max_accelJ * decel_time)
        position = (
                initial_posJ
                + direction*(0.5  * max_accelJ * time_accel ** 2
                + max_veloJ * (time_total - 2 * time_accel)
                - 0.5  * max_accelJ * decel_time ** 2
                + max_veloJ * decel_time)
        )
        acceleration = -max_accelJ * direction
    if current_time >= end_time:
        position = target_posJ
        velocity = 0.0
        acceleration = 0.0
    
    return current_time, position, velocity, acceleration
    

# Visualize the trajectory
def visualize_trajectory(current_time, position, velocity, acceleration):
    plt.figure(figsize=(10, 6))
    # Plot results
    plt.subplot(3, 1, 1)
    plt.plot(current_time, position, label='Position', linewidth=2, color='blue')
    plt.title('Position vs Time')
    plt.xlabel('current_time (sec)')
    plt.ylabel('position (mm)')
    plt.legend()


    plt.subplot(3, 1, 2)
    plt.plot(current_time, velocity, label='Velocity', linewidth=2, color='red')
    plt.title('Velocity vs Time')
    plt.xlabel('current_time (sec)')
    plt.ylabel('Velocity (mm/s)')
    plt.legend()


    plt.subplot(3, 1, 3)
    plt.plot(current_time, acceleration, label='Acceleration', linewidth=2, color='green')
    plt.title('Acceleration vs Time')
    plt.xlabel('current_time (sec)')
    plt.ylabel('Acceleration (mm/s^2)')
    plt.legend()

    plt.tight_layout()
    plt.show()


# Calculate Time
def calcTime(initial_posJ, target_posJ, max_veloJ, max_accelJ):
    # Calculated Distance
    s = abs(target_posJ - initial_posJ)  

    # Define pattern of trapezoidal_trajectory
    if s > (max_veloJ ** 2) / max_accelJ:
    # Calculate the time required for acceleration and deceleration for
        time_accel = max_veloJ / max_accelJ
        time_total = 2 * time_accel + (s - (max_veloJ ** 2) / max_accelJ) / max_veloJ
    else:
    # Calculate the time required for acceleration and deceleration
        time_accel = np.sqrt(s / max_accelJ)
        time_total = 2 * time_accel
    time = np.arange(0, time_total, 0.01)
    
    return  time_total


# Calculate Velocity Max
def calcVelocityConstraint(initial_posJ,target_posJ, max_accelJ, time_max):
    veloCompute = ((max_accelJ * time_max) - np.sqrt(((max_accelJ * time_max)**2 - 4 * np.abs(target_posJ-initial_posJ) * max_accelJ)))/2
    return veloCompute


def calcTimeMax(qi,qf,accel_max):
    # Calculate Time Max of q1, q2, q3
    max_time = np.array([[0,0,0]]).T.astype(float)
    max_time[0][0] = calcTime(qi[0], qf[0][0], 50, accel_max)
    max_time[1][0] = calcTime(qi[1], qf[1][0], 50, accel_max)
    max_time[2][0] = calcTime(qi[2], qf[2][0], 50, accel_max) 
    time_max = max_time.max()
    return time_max

def calcallVelocityConstraint(qi,qf,accel_max,time_max):
    # Calculate Velocity Max of q'1, q'2, q'3
    velo_Constraint = np.array([[0,0,0]]).T.astype(float)
    velo_Constraint[0][0] = calcVelocityConstraint(qi[0],qf[0][0], accel_max, time_max)
    velo_Constraint[1][0] = calcVelocityConstraint(qi[1],qf[1][0], accel_max, time_max)
    velo_Constraint[2][0] = calcVelocityConstraint(qi[2],qf[2][0], accel_max, time_max)
    # print("Velocity Compute :\n",velo_Constraint)
    return velo_Constraint



def traject_gen(qi, qf, velo_Constraint, accel_max, dt, current_time, time_max):
    current_time1, positionJ1, velocityJ1, accelerationJ1 = trajectory_trapezoidal(qi[0], qf[0], velo_Constraint[0][0], accel_max, dt, current_time, time_max)
    current_time2, positionJ2, velocityJ2, accelerationJ2 = trajectory_trapezoidal(qi[1], qf[1], velo_Constraint[1][0], accel_max, dt, current_time, time_max)
    current_time3, positionJ3, velocityJ3, accelerationJ3 = trajectory_trapezoidal(qi[2], qf[2], velo_Constraint[2][0], accel_max, dt, current_time, time_max)

    q_prime = np.array([[positionJ1],[positionJ2],[positionJ3]])
    v_prime = np.array([[velocityJ1],[velocityJ2],[velocityJ3]])
    a_prime = np.array([[accelerationJ1],[accelerationJ2],[accelerationJ3]])
    current_time = np.array([[current_time1],[current_time2],[current_time3]])
    return q_prime, v_prime, a_prime, current_time