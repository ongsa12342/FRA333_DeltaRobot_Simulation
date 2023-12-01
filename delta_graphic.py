from vpython import *
import numpy as np

ball_size = 10
link_size = 5
link_length = 200
frame_size = 346.41

eff_link = 44.95


class BASE:
    def __init__(self, ball_size = ball_size, link_size=link_size, link_length=link_length, frame_size=frame_size):
        # Sphere objects
        self.sphere_object = sphere(pos=vector(0, 0, 0), radius=ball_size, color=color.blue)
        self.sphere_object1 = sphere(pos=vector(0, -link_length, 0), radius=ball_size, color=color.blue)
        self.sphere_object2 = sphere(pos=vector(-link_length * np.cos(np.radians(30)), link_length * np.cos(np.radians(60)), 0), radius=ball_size, color=color.blue)
        self.sphere_object3 = sphere(pos=vector(link_length * np.cos(np.radians(30)), link_length * np.cos(np.radians(60)), 0), radius=ball_size, color=color.blue)

        # Cylinder objects
        self.cylinder_object = cylinder(
            pos=vector(0, -link_size, 0),
            axis=vector(0, -(link_length - 2 * link_size), 0),
            radius=link_size,
            color=color.green
        )

        self.cylinder_object2 = cylinder(
            pos=vector(link_size * np.cos(np.radians(30)), link_size * np.cos(np.radians(60)), 0),
            axis=vector((link_length - 2 * link_size) * np.cos(np.radians(30)),
                        (link_length - 2 * link_size) * np.cos(np.radians(60)), 0),
            radius=5,
            color=color.green
        )

        self.cylinder_object3 = cylinder(
            pos=vector(-link_size * np.cos(np.radians(30)), link_size * np.cos(np.radians(60)), 0),
            axis=vector(-(link_length - 2 * link_size) * np.cos(np.radians(30)),
                        (link_length - 2 * link_size) * np.cos(np.radians(60)), 0),
            radius=5,
            color=color.green
        )

        # Frame objects
        self.frame_1 = cylinder(
            pos=vector(-link_size * np.cos(np.radians(60)), -(link_length - link_size * np.cos(np.radians(30))), 0),
            axis=vector(-frame_size * np.cos(np.radians(60)) + 2 * link_size * np.cos(np.radians(60)),
                        frame_size * np.cos(np.radians(30)) - 2 * link_size * np.cos(np.radians(30)), 0),
            radius=link_size,
            color=color.green
        )

        self.frame_2 = cylinder(
            pos=vector(link_size * np.cos(np.radians(60)), -(link_length - link_size * np.cos(np.radians(30))), 0),
            axis=vector(frame_size * np.cos(np.radians(60)) - 2 * link_size * np.cos(np.radians(60)),
                        frame_size * np.cos(np.radians(30)) - 2 * link_size * np.cos(np.radians(30)), 0),
            radius=link_size,
            color=color.green
        )

        self.frame_3 = cylinder(
            pos=vector(link_length * np.cos(np.radians(30)), link_length * np.cos(np.radians(60)), 0),
            axis=vector(-(frame_size - 2 * link_size), 0, 0),
            radius=link_size,
            color=color.green
        )



class RF:
    def __init__(self, size = 4, link_length = link_length):
        self.size = size
        self.rf1_init_pos = vector(0, -link_length, 0)
        self.rf2_init_pos = vector(link_length * cos(radians(30)), link_length * cos(radians(60)), 0)
        self.rf3_init_pos = vector(-link_length * cos(radians(30)), link_length * cos(radians(60)), 0)

        self.rf1 = cylinder(
            pos=self.rf1_init_pos,
            axis=vector(0, 0, 0),
            radius=size,
            color=color.red
        )

        self.rf2 = cylinder(
            pos=self.rf2_init_pos,
            axis=vector(0, 0, 0),
            radius=size,
            color=color.green
        )

        self.rf3 = cylinder(
            pos=self.rf3_init_pos,
            axis=vector(0, 0, 0),
            radius=size,
            color=color.blue
        )

    def update_positions(self, rf1_pos, rf2_pos, rf3_pos):
        self.rf1.axis = vector(rf1_pos[0],rf1_pos[1],rf1_pos[2]) - self.rf1_init_pos
        self.rf2.axis = vector(rf2_pos[0],rf2_pos[1],rf2_pos[2]) - self.rf2_init_pos
        self.rf3.axis = vector(rf3_pos[0],rf3_pos[1],rf3_pos[2]) - self.rf3_init_pos

class RE:
    def __init__(self, size = 4):
        self.size = size

        self.re1 = cylinder(
            pos=vector(0, 0, 0),
            axis=vector(0, 0, 0),
            radius=size,
            color=color.red
        )

        self.re2 = cylinder(
            pos=vector(0, 0, 0),
            axis=vector(0, 0, 0),
            radius=size,
            color=color.green
        )

        self.re3 = cylinder(
            pos=vector(0, 0, 0),
            axis=vector(0, 0, 0),
            radius=size,
            color=color.blue
        )

    def update_positions(self, rf1_pos, rf2_pos, rf3_pos , pos):
        self.re1.pos = vector(rf1_pos[0],rf1_pos[1],rf1_pos[2])
        self.re2.pos = vector(rf2_pos[0],rf2_pos[1],rf2_pos[2])
        self.re3.pos = vector(rf3_pos[0],rf3_pos[1],rf3_pos[2])

        self.re1.axis = vector(pos[0] , pos[1], pos[2]) -  self.re1.pos + vector(0, -eff_link, 0)
        self.re2.axis = vector(pos[0] , pos[1], pos[2]) -  self.re2.pos + vector(eff_link * np.cos(np.radians(30)), eff_link * np.cos(np.radians(60)), 0)
        self.re3.axis = vector(pos[0] , pos[1], pos[2]) -  self.re3.pos + vector(-eff_link * np.cos(np.radians(30)), eff_link * np.cos(np.radians(60)), 0)

        # print(self.re1.pos.z + self.re1.axis.z,self.re2.pos.z + self.re2.axis.z,self.re3.pos.z + self.re3.axis.z)
        # self.re1.axis = vector(pos[0] , pos[1], pos[2]) -  self.re1.pos + vector(1,1,0)
        # self.re2.axis = vector(pos[0] , pos[1], pos[2]) -  self.re2.pos + vector(1,1,0)
        # self.re3.axis = vector(pos[0] , pos[1], pos[2]) -  self.re3.pos+ vector(1,1,0)