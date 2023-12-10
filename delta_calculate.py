import numpy as np


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

    y3 = (f*np.tan(np.deg2rad(30))/2 + rf*np.cos(np.deg2rad(q_forward[2][0])))*np.sin(np.deg2rad(30))
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
    if (d < 0): return pos1, pos2, pos3, pos0  #// non-existing point

    z0 = -0.5*(b+np.sqrt(d))/a
    x0 = (a1*z0 + b1)/dnm
    y0 = (a2*z0 + b2)/dnm

    pos0 = np.array([[x0, y0, z0]]).T.astype(float)
    pos1 = np.array([[0, y1, z1]]).T.astype(float)
    pos2 = np.array([[x2, y2, z2]]).T.astype(float)
    pos3 = np.array([[x3, y3, z3]]).T.astype(float)
    
    return pos1, pos2, pos3, pos0



def delta_calcAngleYZ(x0, y0, z0):
    y1 = -0.5 * 0.57735 * f # f/2 * tg 30
    y0 -= 0.5 * 0.57735 * e    # shift center to edge
    # z = a + b*y
    a = (x0*x0 + y0*y0 + z0*z0 +rf*rf - re*re - y1*y1)/(2*z0)
    b = (y1-y0)/z0
    # discriminant
    d = -(a+b*y1)*(a+b*y1)+rf*(b*b*rf+rf)
    if (d < 0): return "error" # non-existing point
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
     if(delta_calcAngleYZ(pos[0][0], pos[1][0], pos[2][0]) != "error"):
          q_inv[0][0] = delta_calcAngleYZ(pos[0][0], pos[1][0], pos[2][0])
          q_inv[1][0] = delta_calcAngleYZ(pos[0][0]*np.cos(np.deg2rad(120)) + pos[1][0]*np.sin(np.deg2rad(120)), pos[1][0]*np.cos(np.deg2rad(120))-pos[0][0]*np.sin(np.deg2rad(120)), pos[2][0])  #rotate coords to +120 deg
          q_inv[2][0] = delta_calcAngleYZ(pos[0][0]*np.cos(np.deg2rad(120)) - pos[1][0]*np.sin(np.deg2rad(120)), pos[1][0]*np.cos(np.deg2rad(120))+pos[0][0]*np.sin(np.deg2rad(120)), pos[2][0])  #rotate coords to -120 deg
     else: return "error"

     return q_inv

def find_theta(pos):
    
    A_y1 = -f*np.tan(np.deg2rad(30))/2 
    A_z1 = 0

    A_y2 = f*np.tan(np.deg2rad(30))/2 * np.sin(np.deg2rad(30))
    A_z2 = 0

    A_y3 = f*np.tan(np.deg2rad(30))/2 * np.sin(np.deg2rad(30))
    A_z3 = 0

    C_x1 = pos[0][0]
    C_y1 = pos[1][0] - e*np.tan(np.deg2rad(30))/2
    C_z1 = pos[2][0]

    C_x2 = pos[0][0] + e*np.tan(np.deg2rad(30))/2 * np.cos(np.deg2rad(30))
    C_y2 = pos[1][0] + e*np.tan(np.deg2rad(30))/2 * np.sin(np.deg2rad(30))
    C_z2 = pos[2][0] 

    C_x3 = pos[0][0] - e*np.tan(np.deg2rad(30))/2 * np.cos(np.deg2rad(30))
    C_y3 = pos[1][0] + e*np.tan(np.deg2rad(30))/2 * np.sin(np.deg2rad(30))
    C_z3 = pos[2][0]

    AC1 = np.sqrt((A_y1-C_y1)**2 + (A_z1-C_z1)**2)
    AC2 = np.sqrt((A_y2-C_y2)**2 + (A_z2-C_z2)**2)
    AC3 = np.sqrt((A_y3-C_y3)**2 + (A_z3-C_z3)**2)

    theta2 = np.array([[0, 0, 0]]).T.astype(float)
    theta2[0][0] = np.arccos((AC1**2 - rf**2 - re**2)/(2*rf*re))
    theta2[1][0] = np.arccos((AC2**2 - rf**2 - re**2)/(2*rf*re))
    theta2[2][0] = np.arccos((AC3**2 - rf**2 - re**2)/(2*rf*re))

    theta3 = np.array([[0, 0, 0]]).T.astype(float)
    theta3[0][0] = 90 - np.rad2deg(np.arcsin(C_x1/re))
    theta3[1][0] = 90 - np.rad2deg(np.arcsin(C_x2/re))
    theta3[2][0] = 90 - np.rad2deg(np.arcsin(C_x3/re)) 

    return theta2, theta3


def Jacobian_pose(theta1, theta2, theta3):
    alpha = np.array([[0, 120, 240]]).T
    Jl_v = np.zeros((3,3))
    for i in range(3):
        Jl_v[i][0] = -np.sin(np.deg2rad(theta3[i][0])) * np.cos(np.deg2rad(theta2[i][0] + theta1[i][0])) * np.sin(np.deg2rad(alpha[i][0])) + np.cos(np.deg2rad(theta3[i][0])) * np.cos(np.deg2rad(alpha[i][0]))
        Jl_v[i][1] = np.sin(np.deg2rad(theta3[i][0])) * np.cos(np.deg2rad(theta2[i][0] + theta1[i][0])) * np.cos(np.deg2rad(alpha[i][0])) + np.cos(np.deg2rad(theta3[i][0])) * np.sin(np.deg2rad(alpha[i][0]))
        Jl_v[i][2] = -np.sin(np.deg2rad(theta3[i][0])) * np.sin(np.deg2rad(theta2[i][0] + theta1[i][0])) 

    Ja_v = np.zeros((3,3))
    for i in range(3):
        Ja_v[i][i] = rf * np.sin(np.deg2rad(theta2[i][0])) * np.sin(np.deg2rad(theta3[i][0]))

    return Jl_v, Ja_v


def input2array(theta1,theta2,theta3):
    return np.array([[theta1,theta2,theta3]]).T

def array2theta(theta):
    return theta.T

# q = input2array(90,90,90)


# rf1_pos,rf2_pos,rf3_pos,end_pos = delta_calcForward(q)
# #endpos = [ 0.00000000e+00,-3.12843496e-14,-1.01983087e+03]
# print(array2theta(end_pos))
# q = delta_calcInverse(end_pos)

# print(q)


#input
# qff = np.array([[90, 90, 90]]).T

# dt = 0.01
# current_time = 0

# accel_max = 50
# qi = [0, 0, 0]
# qf = qff.T[0]


# #vis
# pos1, pos2, pos3 = [], [], []
# velo1, velo2, velo3 = [], [], []
# accel1, accel2, accel3 = [], [], []
# time_array = []


# #compute 
# time_max = calcTimeMax(qf,accel_max)
# velo_Constraint = calcallVelocityConstraint(qf,accel_max,time_max)


# while True:
#     # print(current_time)
#     #compute
#     [[positionJ1],[positionJ2],[positionJ3]] , [[velocityJ1],[velocityJ2],[velocityJ3]] ,[[accelerationJ1],[accelerationJ2],[accelerationJ3]],[[current_time1],[current_time2],[current_time3]] = traject_gen(qi, qf, velo_Constraint, accel_max, dt, current_time, time_max)
    
#     #vis
#     pos1.append(positionJ1)
#     pos2.append(positionJ2)
#     pos3.append(positionJ3)
#     velo1.append(velocityJ1)
#     velo2.append(velocityJ2)
#     velo3.append(velocityJ3)
#     accel1.append(accelerationJ1)
#     accel2.append(accelerationJ2)
#     accel3.append(accelerationJ3)
#     time_array.append(current_time)    

#     # Update the time
#     current_time = current_time + dt
#     if current_time > time_max+dt:
#         break

        
# # Visualize the trajectory
# visualize_trajectory(time_array, pos1, velo1, accel1)
# visualize_trajectory(time_array, pos2, velo2, accel2)
# visualize_trajectory(time_array, pos3, velo3, accel3)