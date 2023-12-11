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


way = sphere(pos = vector(0,0,0), radius = 15,color = color.white,emissive=True)

tip = sphere(pos=vector(0,0,0), radius=1,  make_trail=True) 
tip.trail_color = color.green
tip.trail_radius = 3


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

#trajj
#3kg accel = 100m/s**2
dt = 1/60
accel_max = 1000


from time import sleep
des_pos_delay = np.array([[0],[0],[0]])
while True:
    rate(60)
    
    x_des , y_des ,z_des = desBox.getText()
    way.pos = vector(x_des , y_des ,z_des)
    
    des_pos = np.array([[x_des] , [y_des] ,[z_des]])
    rf1_pos,rf2_pos,rf3_pos,pos = delta_calculate.delta_calcForward(q)
    # print(des_pos_delay != des_pos)
    if not np.all(des_pos_delay == des_pos):
        print(delta_calculate.delta_calcInverse(des_pos))
        if delta_calculate.delta_calcInverse(des_pos) != "error":
            q_inv = delta_calculate.delta_calcInverse(des_pos)
            qi = q.T[0]
            qf = q_inv.T[0]

            #compute 
            time_max = calcTimeMax(qi,q_inv,accel_max)
            velo_Constraint = calcallVelocityConstraint(qi,q_inv,accel_max,time_max)
            print(qi, qf)

            tip.clear_trail()
            tip.pos = vector(pos[0][0],pos[1][0],pos[2][0])
            
            for c_time in np.arange(0,time_max,dt):
                print("kuy"*100)
                print("base",base.sphere_object1.pos,base.sphere_object2.pos,base.sphere_object3.pos)
                print("eff",eff.frame1.pos,eff.frame2.pos,eff.frame3.pos)
                q_traj, _, _, _ = traject_gen(qi, qf, velo_Constraint, accel_max, dt, c_time, time_max)
                _,_,_,pos_traj = delta_calculate.delta_calcForward(q_traj)
                print(q_traj)
                theta2_traj, theta3_traj = delta_calculate.find_theta(pos_traj)
                # print(q_traj)
                print("theta2",theta2_traj)
                # print(theta3_traj)
                Jl_v, Ja_v = delta_calculate.Jacobian_pose(q_traj, theta2_traj, theta3_traj)
                
                Sigularity_status = delta_calculate.check_sigularity(Ja_v)
                
                # print("pos_traj:")
                # print(pos_traj)
                # print(pos_traj)
                sleep(1/10000)
                # print(vector(pos_traj[0][0],pos_traj[1][0],pos_traj[2][0]))
                tip.pos = vector(pos_traj[0][0],pos_traj[1][0],pos_traj[2][0])
    
    
    
    if con == True:
        if con_delay == False:
            #input
            current_time = 0
            q_inv = delta_calculate.delta_calcInverse(des_pos)
            qi = q.T[0]
            qf = q_inv.T[0]

            #compute 
            time_max = calcTimeMax(qi,q_inv,accel_max)
            velo_Constraint = calcallVelocityConstraint(qi,q_inv,accel_max,time_max)

            
        desBox.button.background = color.red   
        desBox.button.text = "Stop!"
        
        
        q_traj, v_prime, a_prime, _ = traject_gen(qi, qf, velo_Constraint, accel_max, dt, current_time, time_max)



        #con

        error = q_traj - q
        V_j = Kp*error

        q = q + V_j*dt
        current_time = current_time + dt
        thetaBox.update_positions(q)
        posBox.update_positions(pos)
        

        

        if np.linalg.norm(q_inv - q) < 0.001 :

            con = False
            
    else:
        desBox.button.background = color.white
        desBox.button.text = "Enter"

        
    con_delay = con
    des_pos_delay = des_pos

    rf.update_positions(rf1_pos.reshape(3,),rf2_pos.reshape(3,),rf3_pos.reshape(3,))
    re.update_positions(rf1_pos.reshape(3,),rf2_pos.reshape(3,),rf3_pos.reshape(3,),pos.reshape(3,))
    eff.update_positions(pos.reshape(3,))
    
    wt.text = f'(x, y, z,) = ({pos[0][0]:.2f}, {pos[1][0]:.2f}, {pos[2][0]:.2f})\n' 
    
