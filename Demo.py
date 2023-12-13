from vpython import *
from delta_graphic import RF,RE,EFF , BASE, INPUTBox

import matplotlib.pyplot as plt
from time import sleep
import numpy as np

from trapezoidal_trajectory import TRAPEZOIDAL
from delta_simulation import DELTA_ROBOT_MODEL


# def callback func
def linear_pose_callback():
    global q
    global T
    wlog.text = ''
    x_in,  y_in ,  z_in = posBox.getText()
    p_in = np.array([x_in, y_in, z_in]).reshape(3, 1)/1000

    q_inv, error_status = delta.inverse_pose_kinematic(p_in)

    if error_status == None:
        posBox.button.background = color.white
        
        q = q_inv
        thetaBox.update_positions(np.rad2deg(q))
        delta.joint_setq(q)
        T = np.array([0, 0, 0]).reshape(3,1)
    else:
        if error_status == "no-solution":
            wlog.text = """<h2 style="color: red; font-size: 24px;"> Error : out of workspace</h2>"""
        posBox.button.background = color.red

def angular_pose_callback():
    global q
    global T
    wlog.text = ''
    theta1, theta2, theta3 = thetaBox.getText()
    q = np.deg2rad(np.array([theta1, theta2, theta3]).reshape(3,1))

    *_, p, error_status = delta.forward_pose_kinematic(q)
    posBox.update_positions(p*1000)
    delta.joint_setq(q)
    T = np.array([0, 0, 0]).reshape(3,1)

    if error_status == "no-solution":
        wlog.text = """<h2 style="color: red; font-size: 24px;"> Error : out of workspace</h2>"""

def mode_callback():
    global mode
    if mode == "MoveJ":
        mode = "MoveL"
        modeButton.text = "MoveL"
    else:
        mode = "MoveJ"
        modeButton.text = "MoveJ"

def controller_callback():
    global control_enable
    control_enable = not control_enable


# Delta robot params (mm)
e = 44.95*2*np.sqrt(3) 
f = 200*2*np.sqrt(3) 
re = 800.0
rf = 235.0

# SI unit (mm -> m)
delta = DELTA_ROBOT_MODEL(frame_lenght = f/1000, 
                          endeffector_lenght = e/1000,
                          upper_link = rf/1000, 
                          lower_link = re/1000,
                          upper_link_mass = 1, 
                          lower_link_mass = 0.5,
                          endeffector_mass = 2)

# Draw delta
base = BASE(link_length = f/(2*np.sqrt(3)), 
            frame_size = f/2)
rf = RF(link_length = f/(2*np.sqrt(3)))
re = RE()
eff = EFF(eff_link = e/(2*np.sqrt(3)))

# Camera setup
scene_canvas = canvas.get_selected()
scene_canvas.width = 1080  
scene_canvas.height = 480 

scene_canvas.camera.pos = vector(200, -1100, -100) 
scene_canvas.camera.axis = vector(0, 1800, -550)

# Init delta robot state
# joint space
q = np.array([0, 0, 0]).reshape(3,1)
qd = np.array([0, 0, 0]).reshape(3,1)
qdd = np.array([0, 0, 0]).reshape(3,1)
# Cartesian space
*_, p, _ = delta.forward_pose_kinematic(q)
T = np.array([0, 0, 0]).reshape(3,1)
# Controller params
dt = 1/1000
Kp_v = 50
Kp_p = 300
sum_e = 0

# Trajectory
angular_velocity_max = 1000 * np.pi / 180 # rad/s
linear_velocity_max = 10 # m/s
angular_acceleration_max = 10000 * np.pi / 180 # rad/s^2
linear_acceleration_max = 100 # m/s^2
moveJ = TRAPEZOIDAL(dt, angular_velocity_max, angular_acceleration_max)
moveL = TRAPEZOIDAL(dt, linear_velocity_max, linear_acceleration_max)


wtext(text= """<h1>Delta Robot, Simulation</h1>""")

# user input
wt = wtext(text=None )

wtext(text='\n' )
# Create winput widgets for x, y, and z
wtext(text='\n Position Input :  ' )
posBox = INPUTBox(init_value = p*1000, buttonbind=linear_pose_callback)

# mode button
wtext(text='     ' )
wtext(text='Mode    :  ' )
wtext(text='     ' )

# Init mode
mode = "MoveJ"
mode_delay = "MoveJ"

modeButton = button(bind=mode_callback,text="MoveJ")

wtext(text='\n\n Angle Input    :  ' )
thetaBox = INPUTBox(buttonbind=angular_pose_callback)

wtext(text='     ' )
wtext(text='Destination Input    :  ' )


# Init control enable
control_enable = False
control_enable_delay = False

desBox = INPUTBox(buttonbind=controller_callback)

# Init destination position

des_pos = np.array([0, 0, 0]).reshape(3,1)
des_pos_delay = np.array([0, 0, 0]).reshape(3,1)

waypoint = sphere(pos = vector(0,0,0),
                  radius = 8,
                  color = color.white,
                  emissive = True)

trail = sphere(pos=vector(0,0,0), 
             radius=1,  
             make_trail=True) 
trail.trail_color = color.green
trail.trail_radius = 3
wtext(text='\n')

wlog = wtext(text='', color=color.red)
error_status = None
while True:
    # Frame rate set
    rate(1/dt)

    # update user input
    x_des , y_des ,z_des = desBox.getText()
    des_pos = np.array([x_des , y_des ,z_des]).reshape(3,1)/1000

    # waypoint update
    waypoint.pos = vector(x_des , y_des ,z_des)

    # update linear posistion
    rf1_p, rf2_p, rf3_p, p, _ = delta.forward_pose_kinematic(q)

    # update pose
    wt.text = f'Positions = ({p[0][0]*1000:.2f}, {p[1][0]*1000:.2f}, {p[2][0]*1000:.2f})     Angle = ({np.rad2deg(q[0][0]):.2f}, {np.rad2deg(q[1][0]):.2f}, {np.rad2deg(q[2][0]):.2f})     Tourqe = ({T[0][0]*1000:.2f}, {T[1][0]*1000:.2f}, {T[2][0]*1000:.2f})'

    
    rf.update_positions(rf1_p.reshape(3,)*1000,
                        rf2_p.reshape(3,)*1000,
                        rf3_p.reshape(3,)*1000)
    re.update_positions(rf1_p.reshape(3,)*1000,
                        rf2_p.reshape(3,)*1000,
                        rf3_p.reshape(3,)*1000,
                        p.reshape(3,)*1000)
    eff.update_positions(p.reshape(3,)*1000)

    # trail generate
    if (not np.all(des_pos_delay == des_pos)) or mode != mode_delay:
        # trail reset
        sleep(0.05)
        trail.pos = vector(p[0][0]*1000,
                           p[1][0]*1000,
                           p[2][0]*1000)
        trail.clear_trail()
        sleep(0.05)


        _q , error_status = delta.inverse_pose_kinematic(des_pos)

        if error_status == None:
            
            # traject setup
            if mode == "MoveJ":
                qi = q
                qf = _q
                
                moveJ.path(qi,qf)
                
            else:              
                moveL.path(p,des_pos)
            
            if mode == "MoveJ":
                #Path move J Check
                for c_time in np.arange(0,moveJ.time_max+dt,dt):

                    q_traj, qd_traj, *_ = moveJ.traject_gen(c_time)
                    
                    #find pos of move
                    *_, pos_traj, _ = delta.forward_pose_kinematic(q_traj)

                    #trail update
                    trail.pos = vector(pos_traj[0][0]*1000,
                                       pos_traj[1][0]*1000,
                                       pos_traj[2][0]*1000)
            else:
                
                #Path move L Check
                for c_time in np.arange(0,moveL.time_max+dt,dt):

                    pos_traj, velo_traj, *_ = moveL.traject_gen(c_time)

                    q_traj, error_status = delta.inverse_pose_kinematic(pos_traj)


                    *_, p, status = delta.forward_pose_kinematic(q_traj)
                    
                    
                    if not error_status == None :
                        print(f'moveL IPK : {error_status}')
                        wlog.text = """<h2 style="color: red; font-size: 24px;"> Error : Can not go out off workspace</h2>"""
                        break
          
                    _, error_status = delta.inverse_twist_kinematic(velo_traj, q_traj)
                    if not error_status == None : 
                        print(f'moveL ITK : {error_status}')
                        wlog.text = """<h2 style="color: red; font-size: 24px;"> Error : Can not move Linear (Singularity)</h2>"""
                        break

                    #trail update
                    trail.pos = vector(pos_traj[0][0]*1000,
                                       pos_traj[1][0]*1000,
                                       pos_traj[2][0]*1000)
    if control_enable == True and error_status != None:
        wlog.text = """<h2 style="color: red; font-size: 24px;"> Error : Wrong Input</h2>"""

    # Controller                
    if control_enable == True and error_status == None:
        
        # first time controller enable 
        if control_enable_delay == False:
            #Init trajectory
            current_time = 0
            qi = q
            qf, _ = delta.inverse_pose_kinematic(des_pos)

            if mode == "MoveJ":       
                moveJ.path(qi,qf)
            else:
                moveL.path(p,des_pos)

            #init q
            delta.joint_setq(q)

        desBox.button.background = color.red   
        desBox.button.text = "Stop!"
        
        # trajectory gen
        if mode == "MoveJ":
            q_traj, qd_traj, _, _ = moveJ.traject_gen(current_time)
        else:
            pos_traj ,  velo_traj, _, _ = moveL.traject_gen(current_time)
            q_traj,  _ = delta.inverse_pose_kinematic(pos_traj)
            qd_traj, _ = delta.inverse_twist_kinematic(velo_traj, q_traj)



        position_error = q_traj - q
        V = Kp_p * position_error

        velocity_error = qd_traj - qd + V

        T = Kp_v * velocity_error


        # update
        current_time = current_time + dt

        thetaBox.update_positions(np.rad2deg(q))
        posBox.update_positions(p*1000)
        
        if mode == "MoveJ":
            if (current_time > moveJ.time_max + 0.5):
                control_enable = False
        else:
            if (current_time > moveL.time_max + 0.5):
                control_enable = False
                
    else:
        desBox.button.background = color.white
        desBox.button.text = "Enter"
        control_enable = False
        


    # dynamic model
    q, qd, qdd = delta.dynamic_model(T, dt)

    
    # delay value
    mode_delay = mode
    control_enable_delay = control_enable
    des_pos_delay = des_pos
