from vpython import *
from time import *
import math
import delta_calculate
from delta_graphic import RF,RE , BASE
import numpy as np


base = BASE()
rf = RF()
re = RE()

scene_canvas = canvas.get_selected()
scene_canvas.width = 1080  
scene_canvas.height = 720 

angle1 = 90
angle2 = 45
angle3 = 0
da1 = 1
da2 = 1
da3 = 1

while True:
    rate(60)
    if angle1 > 90:
        da1 = -da1
    elif angle1 < 0:
        da1 = -da1

    
    if angle2 > 90:
        da2 = -da2
    elif angle2 < 0:
        da2 = -da2
    
    if angle3 > 90:
        da3 = -da3
    elif angle3 < 0:
        da3 = -da3

    # print(angle)
    rf1_pos,rf2_pos,rf3_pos,pos = delta_calculate.delta_calcForward(angle1, angle2,angle3)
    rf.update_positions(rf1_pos,rf2_pos,rf3_pos)
    re.update_positions(rf1_pos,rf2_pos,rf3_pos,pos)

    angle1 += da1
    angle2 += da2
    angle3 += da3