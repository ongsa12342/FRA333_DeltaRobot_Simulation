import numpy as np
import matplotlib.pyplot as plt

class TRAPEZOIDAL:
    def __init__(self,dt, velocity_max, acceleration_max):
        self.acceleration_max = acceleration_max
        self.dt = dt
        self.qi = None
        self.qf = None
        self.time_max = None
        self.velocity_Constraint = None
        self.velocity_max = velocity_max

    def path(self,qi,qf):
        self.qi = qi
        self.qf = qf

        self.time_max = calcTimeMax(qi,qf, self.velocity_max,self.acceleration_max)
        self.velocity_Constraint = calcallVelocityConstraint(qi,qf,self.acceleration_max,self.time_max)
    def traject_gen(self, current_time):
        positionJ1, velocityJ1, accelerationJ1 = trajectory_trapezoidal(self.qi[0][0], self.qf[0][0], self.velocity_Constraint[0][0], self.acceleration_max, current_time)
        positionJ2, velocityJ2, accelerationJ2 = trajectory_trapezoidal(self.qi[1][0], self.qf[1][0], self.velocity_Constraint[1][0], self.acceleration_max, current_time)
        positionJ3, velocityJ3, accelerationJ3 = trajectory_trapezoidal(self.qi[2][0], self.qf[2][0], self.velocity_Constraint[2][0], self.acceleration_max, current_time)

        q_traj = np.array([[positionJ1],[positionJ2],[positionJ3]])
        v_traj = np.array([[velocityJ1],[velocityJ2],[velocityJ3]])
        a_traj = np.array([[accelerationJ1],[accelerationJ2],[accelerationJ3]])
        return q_traj,  v_traj, a_traj





def trajectory_trapezoidal(initial_posJ, target_posJ, max_veloJ, max_accelJ, current_time):
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
    elif time_total - time_accel <= current_time < time_total:
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
    if current_time >= time_total:
        position = target_posJ
        velocity = 0.0
        acceleration = 0.0
    
    return position, velocity, acceleration
    

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
        time_accel = np.sqrt(2*s / max_accelJ)
        time_total = 2 * time_accel
    
    return  time_total


# Calculate Velocity Max
def calcVelocityConstraint(initial_posJ,target_posJ, max_accelJ, time_max):
    # print(time_max)
    # print(max_accelJ)
    # print(np.abs(target_posJ-initial_posJ))
    # print("sqrt",(max_accelJ * time_max)**2 - 4 * np.abs(target_posJ-initial_posJ) * max_accelJ)
    # print((max_accelJ * time_max)**2)
    # print(4 * np.abs(target_posJ-initial_posJ) * max_accelJ)
    veloCompute = ((max_accelJ * time_max) - np.sqrt((max_accelJ * time_max)**2 - 4 * np.abs(target_posJ-initial_posJ) * max_accelJ))/2
    # print("velocityCompute:", veloCompute)
    return veloCompute


def calcTimeMax(qi,qf,vel_max,accel_max):
    # Calculate Time Max of q1, q2, q3
    max_time = np.array([[0,0,0]]).T.astype(float)
    max_time[0][0] = calcTime(qi[0][0], qf[0][0], vel_max, accel_max)
    max_time[1][0] = calcTime(qi[1][0], qf[1][0], vel_max, accel_max)
    max_time[2][0] = calcTime(qi[2][0], qf[2][0], vel_max, accel_max) 
    time_max = max_time.max()
    # print("TimeMax:", time_max)
    return time_max

def calcallVelocityConstraint(qi,qf,accel_max,time_max):
    # Calculate Velocity Max of q'1, q'2, q'3
    velo_Constraint = np.array([[0,0,0]]).T.astype(float)
    velo_Constraint[0][0] = calcVelocityConstraint(qi[0][0],qf[0][0], accel_max, time_max)
    velo_Constraint[1][0] = calcVelocityConstraint(qi[1][0],qf[1][0], accel_max, time_max)
    velo_Constraint[2][0] = calcVelocityConstraint(qi[2][0],qf[2][0], accel_max, time_max)
    # print("Velocity Compute :\n",velo_Constraint)
    return velo_Constraint





# max_accelJ = 1
# time_max = 0.6456360795584286
# veloCompute = ((max_accelJ * time_max) - np.sqrt((max_accelJ * time_max)**2 - 4 * np.abs(0.1042114868068944) * max_accelJ))/2
# print(veloCompute)