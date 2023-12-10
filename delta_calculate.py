import numpy as np
import time
from trajectoryNew import *


# e = 0.04495*2*np.sqrt(3) #155.71136760044206     #end effector #115.0
# f = 0.2*2*np.sqrt(3)   #346.41016151377545  #base #457.3
# re = 0.8 #232.0
# rf = 0.235 #112.0

e = 44.95*2*np.sqrt(3) #155.71136760044206     #end effector #115.0
f = 200*2*np.sqrt(3)   #346.41016151377545  #base #457.3
re = 800.0 #232.0
rf = 235.0 #112.0

def delta_calcForward(q_forward, e = e, f = f  , re = re , rf = rf ):

    t = (f-e)*np.tan(np.deg2rad(30))/2
    # t = 200
    #dtr = pi/180.0

    # q_forward[0][0] = np.deg2rad(q_forward[0])
    # q_forward[1][0] = np.deg2rad(q_forward[1])
    # q_forward[2][0] = np.deg2rad(q_forward[2])

    y1 = -(f*np.tan(np.deg2rad(30))/2 + rf*np.cos(np.deg2rad(q_forward[0][0])))
    z1 = -rf*np.sin(np.deg2rad(q_forward[0][0]))

    y2 = (f*np.tan(np.deg2rad(30))/2 + rf*np.cos(np.deg2rad(q_forward[1][0])))*np.sin(np.deg2rad(30))
    x2 = y2*np.tan(np.deg2rad(60))
    z2 = -rf*np.sin(np.deg2rad(q_forward[1][0]))

    y3 = (f*np.tan(np.deg2rad(30))/2 + rf*np.cos(np.deg2rad(q_forward[2])))*np.sin(np.deg2rad(30))
    x3 = -y3*np.tan(np.deg2rad(60))
    z3 = -rf*np.sin(np.deg2rad(q_forward[2][0]))

    y1_e = -(t + rf*np.cos(np.deg2rad(q_forward[0][0])))
    z1_e = -rf*np.sin(np.deg2rad(q_forward[0][0]))

    y2_e = (t + rf*np.cos(np.deg2rad(q_forward[1][0])))*np.sin(np.deg2rad(30))
    x2_e = y2_e*np.tan(np.deg2rad(60))
    z2_e = -rf*np.sin(np.deg2rad(q_forward[1][0]))

    y3_e = (t + rf*np.cos(np.deg2rad(q_forward[2][0])))*np.sin(np.deg2rad(30))
    x3_e = -y3_e*np.tan(np.deg2rad(60))
    z3_e = -rf*np.sin(np.deg2rad(q_forward[2][0]))

    dnm = (y2_e-y1_e)*x3_e-(y3_e-y1_e)*x2_e

    w1 = y1_e*y1_e + z1_e*z1_e
    w2 = x2_e*x2_e + y2_e*y2_e + z2_e*z2_e
    w3 = x3_e*x3_e + y3_e*y3_e + z3_e*z3_e
    
    #x = (a1*z + b1)/dnm
    a1 = (z2_e-z1_e)*(y3_e-y1_e)-(z3_e-z1_e)*(y2_e-y1_e)
    b1 = -((w2-w1)*(y3_e-y1_e)-(w3-w1)*(y2_e-y1_e))/2.0

    #y = (a2*z + b2)/dnm
    a2 = -(z2_e-z1_e)*x3_e+(z3_e-z1_e)*x2_e
    b2 = ((w2-w1)*x3_e - (w3-w1)*x2_e)/2.0

    #a*z^2 + b*z + c = 0
    a = a1*a1 + a2*a2 + dnm*dnm
    b = 2*(a1*b1 + a2*(b2-y1_e*dnm) - z1_e*dnm*dnm)
    c = (b2-y1_e*dnm)*(b2-y1_e*dnm) + b1*b1 + dnm*dnm*(z1_e*z1_e - re*re)

    #discriminant
    d = b*b - 4.0*a*c
    if (d < 0): return -1 #// non-existing point

    z0 = -0.5*(b+np.sqrt(d))/a
    x0 = (a1*z0 + b1)/dnm
    y0 = (a2*z0 + b2)/dnm

    pos0 = np.array([[x0, y0, z0]]).T.astype(float)
    
    return [0, y1, z1], [x2, y2, z2], [x3, y3, z3] , pos0



def delta_calcAngleYZ(x0, y0, z0):
    y1 = -0.5 * 0.57735 * f # f/2 * tg 30
    y0 -= 0.5 * 0.57735 * e    # shift center to edge
    # z = a + b*y
    a = (x0*x0 + y0*y0 + z0*z0 +rf*rf - re*re - y1*y1)/(2*z0)
    b = (y1-y0)/z0
    # discriminant
    d = -(a+b*y1)*(a+b*y1)+rf*(b*b*rf+rf)
    if (d < 0): return -999.0 # non-existing point
    yj = (y1 - a*b - np.sqrt(d))/(b*b + 1) # choosing outer point
    zj = a + b*yj
    y_offset = 180.0 if yj > y1 else 0.0
    theta = np.degrees(np.arctan2(-zj, (y1 - yj))) + y_offset
    return theta

#inverse kinematics: (x0, y0, z0) -> (q_forward[0][0][0][0], q_forward[1][0], q_forward[2][0])
#returned status: 0=OK, -1=non-existing position
#inverse kinematics: (x0, y0, z0) -> (q_forward[0][0][0], q_forward[0][0][1], q_forward[0][0][2])
#returned status: 0=OK, -1=non-existing position
def delta_calcInverse(pos):
     q_inv = np.array([[0, 0, 0]]).T.astype(float)
     if(delta_calcAngleYZ(pos[0][0], pos[1][0], pos[2][0]) != 0.0):
          q_inv[0][0] = delta_calcAngleYZ(pos[0][0], pos[1][0], pos[2][0])
          q_inv[1][0] = delta_calcAngleYZ(pos[0][0]*np.cos(np.deg2rad(120)) + pos[1][0]*np.sin(np.deg2rad(120)), pos[1][0]*np.cos(np.deg2rad(120))-pos[0][0]*np.sin(np.deg2rad(120)), pos[2][0])  #rotate coords to +120 deg
          q_inv[2][0] = delta_calcAngleYZ(pos[0][0]*np.cos(np.deg2rad(120)) - pos[1][0]*np.sin(np.deg2rad(120)), pos[1][0]*np.cos(np.deg2rad(120))+pos[0][0]*np.sin(np.deg2rad(120)), pos[2][0])  #rotate coords to -120 deg
     else: return -999.0

     return q_inv



#input
q = np.array([[90, 90, 90]]).T

dt = 0.01
current_time = 0

accel_max = 50
qi = [0, 0, 0]
qf = q.T[0]


#vis
pos1, pos2, pos3 = [], [], []
velo1, velo2, velo3 = [], [], []
accel1, accel2, accel3 = [], [], []
time_array = []


#compute 
time_max = calcTimeMax(q,accel_max)
velo_Constraint = calcallVelocityConstraint(q,accel_max,time_max)


while True:
    # print(current_time)
    #compute
    [[positionJ1],[positionJ2],[positionJ3]] , [[velocityJ1],[velocityJ2],[velocityJ3]] ,[[accelerationJ1],[accelerationJ2],[accelerationJ3]],[[current_time1],[current_time2],[current_time3]] = traject_gen(qi, qf, velo_Constraint, accel_max, dt, current_time, time_max)
    
    #vis
    pos1.append(positionJ1)
    pos2.append(positionJ2)
    pos3.append(positionJ3)
    velo1.append(velocityJ1)
    velo2.append(velocityJ2)
    velo3.append(velocityJ3)
    accel1.append(accelerationJ1)
    accel2.append(accelerationJ2)
    accel3.append(accelerationJ3)
    time_array.append(current_time)    

    # Update the time
    current_time = current_time + dt
    time.sleep(dt)
    if current_time > time_max+dt:
        break

        
# Visualize the trajectory
visualize_trajectory(time_array, pos1, velo1, accel1)
visualize_trajectory(time_array, pos2, velo2, accel2)
visualize_trajectory(time_array, pos3, velo3, accel3)
