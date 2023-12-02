from vpython import *
from time import *
import math
import delta_calculate
from delta_graphic import RF,RE,EFF , BASE
import numpy as np
from time import sleep

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



_,_,_,[x_in, y_in, z_in] = delta_calculate.delta_calcForward(0, 0, 0)


def in_theta_update():
    in_theta0.text = f'{theta1:.2f}'
    in_theta1.text = f'{theta2:.2f}'
    in_theta2.text = f'{theta3:.2f}'

def in_pos_update(p):
    in_x.text = f'{p[0]:.2f}'
    in_y.text = f'{p[1]:.2f}'
    in_z.text = f'{p[2]:.2f}'

wtext(text='\n' )
wt = wtext(text='\n' )
# Create winput widgets for x, y, and z
wtext(text='\n Position Input :  ' )


in_x = winput(bind= lambda: None, prompt='x',text=f'{x_in:.2f}')
in_y = winput(bind= lambda: None, prompt='y',text=f'{y_in:.2f}')
in_z = winput(bind= lambda: None, prompt='z',text=f'{z_in:.2f}')
wtext(text='       ' )


def button_get():
    global theta1
    global theta2
    global theta3

    if delta_calculate.delta_calcInverse(float(in_x.text), float(in_y.text),float(in_z.text)) == "non-existing point":
        calc_button.background = color.red
        
    else:
        calc_button.background = color.white
        x_in = float(in_x.text)
        y_in = float(in_y.text)
        z_in = float(in_z.text)
        theta1, theta2, theta3 = delta_calculate.delta_calcInverse(x_in, y_in,z_in)
        
        in_theta_update()



calc_button = button(bind=button_get,text="Enter")


wtext(text='     ' )
wtext(text='Destination Input    :  ' )
des_x = winput(text = 0 , bind= lambda: None, prompt='x')
des_y = winput(text = 0 ,bind= lambda: None, prompt='y')
des_z = winput(text = 0 ,bind= lambda: None, prompt='z')
way = sphere(pos = vector(0,0,0), radius = 10,color = color.blue,emissive=True)
way.emissive = 1
traject = []
def traj():
    global traject

    # Specify the start and end values for each dimension
    x_start, x_end = pos[0], float(des_x.text)
    y_start, y_end = pos[1], float(des_y.text)
    z_start, z_end = pos[2], float(des_z.text)

    # Specify the number of points
    num_points = 120  # Adjust this as needed

    # Generate linearly spaced values for each dimension
    x_values = np.linspace(x_start, x_end, num_points)
    y_values = np.linspace(y_start, y_end, num_points)
    z_values = np.linspace(z_start, z_end, num_points)

    # Combine the values into a single array
    traject = np.column_stack((x_values, y_values, z_values))

go_button = button(bind=traj,text="Enter")


wtext(text='\n\n Angle Input    :  ' )
in_theta0 = winput(bind= lambda: None, prompt='x',text=f'{0:.2f}')
in_theta1 = winput(bind= lambda: None, prompt='y',text=f'{0:.2f}')
in_theta2 = winput(bind= lambda: None, prompt='z',text=f'{0:.2f}')
wtext(text='       ' )

def theta_get():
    global theta1
    global theta2
    global theta3

    theta1, theta2, theta3 = float(in_theta0.text), float(in_theta1.text),float(in_theta2.text)
    _,_,_, p = delta_calculate.delta_calcForward(theta1, theta2, theta3)

theta_button = button(bind=theta_get,text="Enter")

while True:
    rate(60)
    if len(traject) > 0:
        print(traject)
        theta1,theta2,theta3 = delta_calculate.delta_calcInverse( traject[0][0],traject[0][1],  traject[0][2])
        traject = traject[1:]
        in_theta_update()
        in_pos_update(pos)
        

    way.pos = vector(float(des_x.text),float(des_y.text),float(des_z.text))
    rf1_pos,rf2_pos,rf3_pos,pos = delta_calculate.delta_calcForward(theta1, theta2, theta3)
    
    rf.update_positions(rf1_pos,rf2_pos,rf3_pos)
    re.update_positions(rf1_pos,rf2_pos,rf3_pos,pos)
    eff.update_positions(pos)
    
    wt.text = f'(x, y, z,) = ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})\n' 
    
    
