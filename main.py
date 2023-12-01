from vpython import *
from time import *
import math
import delta_calculate
from delta_graphic import RF,RE,EFF , BASE
import numpy as np


base = BASE()
rf = RF()
re = RE()
eff = EFF()
scene_canvas = canvas.get_selected()
scene_canvas.width = 1080  
scene_canvas.height = 480 


scene_canvas.camera.pos = vector(200, -1100, -100) 
scene_canvas.camera.axis = vector(0, 1800, -550)


# # Function to handle keyboard input for zooming
# def zoom(event):
#     if event.key == 'up':
#         scene.fov *= 1.1  # Zoom in
#     elif event.key == 'down':
#         scene.fov /= 1.1  # Zoom out

# # Bind the zoom function to the keyboard events
# scene.bind('keydown', zoom)



angle1 = 90
angle2 = 45
angle3 = 0
da1 = 1
da2 = 1
da3 = 1

x_in = 0
y_in = 0
z_in = 0


def on_input_x(w):
    global x_in
    x_in = float(w.text)  


def on_input_y(w):
    global y_in
    y_in = float(w.text) 


def on_input_z(w):
    global z_in
    z_in = float(w.text)  





wt = wtext(text='\n' )
# Create winput widgets for x, y, and z
wtext(text='\n' )
in_x = winput(bind=on_input_x, prompt='x')
in_y = winput(bind=on_input_y, prompt='y')
in_z = winput(bind=on_input_z, prompt='z')
# Input box for the number
# input_box = winput(bind=None, prompt='Enter a number:', type='numeric', width=100, pos=scene.title_anchor)




while True:
    rate(60)
    if angle1 > 120:
        da1 = -da1
    elif angle1 < -45:
        da1 = -da1

    
    if angle2 > 120:
        da2 = -da2
    elif angle2 < -45:
        da2 = -da2
    
    if angle3 > 120:
        da3 = -da3
    elif angle3 < -45:
        da3 = -da3


    rf1_pos,rf2_pos,rf3_pos,pos = delta_calculate.delta_calcForward(x_in, y_in,z_in)
    rf.update_positions(rf1_pos,rf2_pos,rf3_pos)
    re.update_positions(rf1_pos,rf2_pos,rf3_pos,pos)
    eff.update_positions(pos)

    
    wt.text = f'(x, y, z,) = ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})\n' 
    # print(x_in,y_in,z_in)

    angle1 += da1
    angle2 += da2
    angle3 += da3