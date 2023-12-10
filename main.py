from vpython import *
from time import *
import math
import delta_calculate
from delta_graphic import RF,RE,EFF , BASE, INPUTBox
import numpy as np
from time import sleep
from trajectoryNew import *
base = BASE()
rf = RF()
re = RE()
eff = EFF()

scene_canvas = canvas.get_selected()
scene_canvas.width = 1080  
scene_canvas.height = 480 


scene_canvas.camera.pos = vector(200, -1100, -100) 
scene_canvas.camera.axis = vector(0, 1800, -550)

theta1 = 0
theta2 = 0
theta3 = 0
q = delta_calculate.input2array(theta1, theta2, theta3)
wtext(text='\n' )
wt = wtext(text='\n' )
# Create winput widgets for x, y, and z
wtext(text='\n Position Input :  ' )

_,_,_,init_pos = delta_calculate.delta_calcForward(delta_calculate.input2array(0,0,0))

def pos_get():
    global q
    x_in,  y_in ,  z_in = posBox.getText()
    if delta_calculate.delta_calcInverse(delta_calculate.input2array(x_in, y_in,z_in)) != "error":
        posBox.button.background = color.white
        
        q = delta_calculate.delta_calcInverse(delta_calculate.input2array(x_in, y_in,z_in))
        thetaBox.update_positions(q)
    else:
        posBox.button.background = color.red


posBox = INPUTBox(init_value = init_pos,buttonbind=pos_get)


wtext(text='     ' )
wtext(text='Destination Input    :  ' )
con = False
con_delay = False
def con_get():
    global con
    con = not con

desBox = INPUTBox(buttonbind=con_get)
desBox.c.text = -900

way = sphere(pos = vector(0,0,0), radius = 15,color = color.white,emissive=True)




wtext(text='\n\n Angle Input    :  ' )



def theta_get():
    global q

    theta1, theta2, theta3 = thetaBox.getText()
    q = delta_calculate.input2array(theta1, theta2, theta3)
    _,_,_, p = delta_calculate.delta_calcForward(delta_calculate.input2array(theta1, theta2, theta3))
    posBox.update_positions(p)

thetaBox = INPUTBox(buttonbind=theta_get)
dt = 1/60
Kp = 10
Ki = 0
sum_e = 0




#traj




while True:
    rate(60)
    
    x_des , y_des ,z_des = desBox.getText()
    way.pos = vector(x_des , y_des ,z_des)
    des_pos = np.array([[x_des] , [y_des] ,[z_des]])
    rf1_pos,rf2_pos,rf3_pos,pos = delta_calculate.delta_calcForward(q)
    
    if con == True:
        if con_delay == False:
            #input

            dt = 1/60
            current_time = 0

            accel_max = 50
            qi = q.T[0]
            qf = des_pos.T[0]

            #compute 
            time_max = calcTimeMax(des_pos,accel_max)
            velo_Constraint = calcallVelocityConstraint(des_pos,accel_max,time_max)
            
        desBox.button.background = color.red
        desBox.button.text = "Stop!"
        theta2_np, theta3_np = delta_calculate.find_theta(pos)

        # error = des_pos - pos
        # sum_e = sum_e + error
        # V_e = Kp*error #+ Ki*sum_e*dt

        # Jp, Jt = delta_calculate.Jacobian_pose(q,theta2_np,theta3_np)
        # Jt_inv = np.linalg.inv(Jt)

        # qd = np.dot(Jt_inv,np.dot(Jp,V_e))

        # q = q + qd*dt
        
        q, v_prime, a_prime, _ = traject_gen(qi, qf, velo_Constraint, accel_max, dt, current_time, time_max)
        current_time = current_time + dt
        thetaBox.update_positions(q)
        posBox.update_positions(pos)

        if  current_time > time_max+dt:
            con = False
            
    else:
        desBox.button.background = color.white
        desBox.button.text = "Enter"

        
    con_delay = con
        

    rf.update_positions(rf1_pos.reshape(3,),rf2_pos.reshape(3,),rf3_pos.reshape(3,))
    re.update_positions(rf1_pos.reshape(3,),rf2_pos.reshape(3,),rf3_pos.reshape(3,),pos.reshape(3,))
    eff.update_positions(pos.reshape(3,))
    
    wt.text = f'(x, y, z,) = ({pos[0][0]:.2f}, {pos[1][0]:.2f}, {pos[2][0]:.2f})\n' 
    
