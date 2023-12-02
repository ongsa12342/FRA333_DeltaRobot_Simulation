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




angle1 = 90
angle2 = 45
angle3 = 0
da1 = 1
da2 = 1
da3 = 1

_,_,_,[x_in, y_in, z_in] = delta_calculate.delta_calcForward(0, 0, 0)


wt = wtext(text='\n' )
# Create winput widgets for x, y, and z
wtext(text='\n' )
in_x = winput(bind= lambda: None, prompt='x')
in_y = winput(bind= lambda: None, prompt='y')
in_z = winput(bind= lambda: None, prompt='z')
wtext(text='       ' )


def button_get():
    global x_in
    global y_in
    global z_in

    if delta_calculate.delta_calcInverse(float(in_x.text), float(in_y.text),float(in_z.text)) == "non-existing point":
        calc_button.background = color.red
        
    else:
        calc_button.background = color.white
        x_in = float(in_x.text)
        y_in = float(in_y.text)
        z_in = float(in_z.text)



calc_button = button(bind=button_get,text="Calculate")




while True:
    rate(60)
    theta1, theta2, theta3 = delta_calculate.delta_calcInverse(x_in, y_in,z_in)
    rf1_pos,rf2_pos,rf3_pos,pos = delta_calculate.delta_calcForward(theta1, theta2, theta3)
    rf.update_positions(rf1_pos,rf2_pos,rf3_pos)
    re.update_positions(rf1_pos,rf2_pos,rf3_pos,pos)
    eff.update_positions(pos)
    
    wt.text = f'(x, y, z,) = ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})\n' 
    # print(x_in,y_in,z_in)

    angle1 += da1
    angle2 += da2
    angle3 += da3