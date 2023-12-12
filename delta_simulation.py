import numpy as np

class delta_robot_model:
    def __init__(self, frame_lenght, endeffector_lenght, 
                 upper_link, lower_link, upper_link_mass, 
                 lower_link_mass, endeffector_mass):
        # user define parameter
        self.f = frame_lenght
        self.e = endeffector_lenght
        self.rf = upper_link
        self.re = lower_link
        self.m = [endeffector_mass, upper_link_mass, lower_link_mass]

        # robot constant
        self.r = (self.f-self.e)/(2*np.sqrt(3))
        self.alpha = [0 - 90, 120 - 90, 240 - 90]

        # state in joint space
        self.q = np.array([0, 0, 0]).reshape(3, 1)
        self.qd = np.array([0, 0, 0]).reshape(3, 1)
        self.qdd = np.array([0, 0, 0]).reshape(3, 1)

        # joint position
        self.p1 = np.array([0, 0, 0]).reshape(3, 1)
        self.p2 = np.array([0, 0, 0]).reshape(3, 1)
        self.p3 = np.array([0, 0, 0]).reshape(3, 1) 
    
    def forward_pose_kinematic(self, q : np):
        t = (self.f-self.e)*np.tan(np.deg2rad(30))/2

        y1 = -(self.f*np.tan(np.deg2rad(30))/2 + self.rf*np.cos(q[0][0]))
        z1 = -self.rf*np.sin(q[0][0])

        y2 = (self.f*np.tan(np.deg2rad(30))/2 + self.rf*np.cos(q[1][0]))*np.sin(np.deg2rad(30))
        x2 = y2*np.tan(np.deg2rad(60))
        z2 = -self.rf*np.sin(q[1][0])

        y3 = (self.f*np.tan(np.deg2rad(30))/2 + self.rf*np.cos(q[2][0]))*np.sin(np.deg2rad(30))
        x3 = -y3*np.tan(np.deg2rad(60))
        z3 = -self.rf*np.sin(q[2][0])

        y1_e = -(t + self.rf*np.cos(q[0][0]))
        z1_e = -self.rf*np.sin(q[0][0])

        y2_e = (t + self.rf*np.cos(q[1][0]))*np.sin(np.deg2rad(30))
        x2_e = y2_e*np.tan(np.deg2rad(60))
        z2_e = -self.rf*np.sin(q[1][0])

        y3_e = (t + self.rf*np.cos(q[2][0]))*np.sin(np.deg2rad(30))
        x3_e = -y3_e*np.tan(np.deg2rad(60))
        z3_e = -self.rf*np.sin(q[2][0])

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
        c = (b2-y1_e*dnm)*(b2-y1_e*dnm) + b1*b1 + dnm*dnm*(z1_e*z1_e - self.re*self.re)

        #discriminant
        d = b*b - 4.0*a*c
        if (d < 0): 
            pos0 = np.array([[0, 0, 0]]).T.astype(float)
            pos1 = np.array([[0, 0, 0]]).T.astype(float)
            pos2 = np.array([[0, 0, 0]]).T.astype(float)
            pos3 = np.array([[0, 0, 0]]).T.astype(float)
            is_error = "no-solution"
            #// non-existing point
        else:
            z0 = -0.5*(b+np.sqrt(d))/a
            x0 = (a1*z0 + b1)/dnm
            y0 = (a2*z0 + b2)/dnm

            pos0 = np.array([[x0, y0, z0]]).T.astype(float)
            pos1 = np.array([[0, y1, z1]]).T.astype(float)
            pos2 = np.array([[x2, y2, z2]]).T.astype(float)
            pos3 = np.array([[x3, y3, z3]]).T.astype(float)
            is_error = "none"
        return pos1, pos2, pos3, pos0, is_error

    def inverse_pose_kinematic(self, p : np):
        q_inv = np.array([[0, 0, 0]]).T.astype(float)
        status = [False, False, False]
        if(self.calcAngleYZ(p[0][0], p[1][0], p[2][0])[1]):
            q_inv[0][0] = self.calcAngleYZ(p[0][0], p[1][0], p[2][0])
            status[0] = True
        if(self.calcAngleYZ(p[0][0]*np.cos(np.deg2rad(120)) + p[1][0]*np.sin(np.deg2rad(120)), p[1][0]*np.cos(np.deg2rad(120))-p[0][0]*np.sin(np.deg2rad(120)), p[2][0])[1]):
            q_inv[1][0] = self.calcAngleYZ(p[0][0]*np.cos(np.deg2rad(120)) + p[1][0]*np.sin(np.deg2rad(120)), p[1][0]*np.cos(np.deg2rad(120))-p[0][0]*np.sin(np.deg2rad(120)), p[2][0])  #rotate coords to +120 deg
            status[1] = True
        if(self.calcAngleYZ(p[0][0]*np.cos(np.deg2rad(120)) - p[1][0]*np.sin(np.deg2rad(120)), p[1][0]*np.cos(np.deg2rad(120))+p[0][0]*np.sin(np.deg2rad(120)), p[2][0])[1]):
            q_inv[2][0] = self.calcAngleYZ(p[0][0]*np.cos(np.deg2rad(120)) - p[1][0]*np.sin(np.deg2rad(120)), p[1][0]*np.cos(np.deg2rad(120))+p[0][0]*np.sin(np.deg2rad(120)), p[2][0])  #rotate coords to -120 deg
            status[2] = True
        if not (status[0] and status[1] and status[2]): is_error = "no-solution"
        else: is_error = "none"
        return q_inv, is_error
    
    def forward_twist_kinematic(self, qd : np, q : np):
        *_, p, status = self.forward_pose_kinematic(q)
        pd = np.array([0, 0, 0]).reshape(3, 1)
        if status:
            theta2, theta3 = self.find_theta(p)
            Jl, Ja = self.Jacobian(q, theta2, theta3)
            if not self.check_singularity(Ja):
                pd = np.linalg.inv(Jl) @ Ja @ qd
                is_error = "none"
            else: is_error = "singularity"
        else: is_error = "no-solution"
        return pd, is_error

    def inverse_twist_kinematic(self, pd : np, q : np):
        *_, p, status = self.forward_pose_kinematic(q)
        qd = np.array([0, 0, 0]).reshape(3, 1)
        if status:
            theta2, theta3 = self.find_theta(p)
            Jl, Ja = self.Jacobian(q, theta2, theta3)
            if not self.check_singularity(Ja):
                qd = np.linalg.inv(Ja) @ Jl @ pd
                is_error = "none"
            else: is_error = "singularity"
        else: is_error = "no-solution"
        return qd, is_error
    
    def dynamic_model(self, T : np, dt):
        I = (self.rf ** 2) * np.array([(self.m[1] / 3) + (self.m[2] / 2), 0, 0,
                                        0, (self.m[1] / 3) + (self.m[2] / 2), 0,
                                        0, 0, (self.m[1] / 3) + (self.m[2] / 2)]).reshape(3, 3)
        G = np.array([0, 0, 0]).reshape(3, 1)
        for i in range(3):
            G[i] = (self.m[1] + self.m[2]) * -9.81 * self.rf * np.cos(self.q[i]) / 2.0
        B = 0.002

        self.qdd = np.linalg.inv(I) @ (T - (B * self.qd) - G)
        self.qd = self.qd + (self.qdd * dt)
        self.q = self.q + (self.qd * dt)

        return self.q, self.qd, self.qdd
    
    def Jacobian(self, q, theta2, theta3):
        alpha = np.deg2rad(np.array([[0, 120, 240]]).T)
        Jl_v = np.zeros((3,3))
        for i in range(3):
            Jl_v[i][0] = -np.sin(theta3[i][0]) * np.cos(theta2[i][0] + q[i][0]) * np.sin(alpha[i][0]) + np.cos(theta3[i][0]) * np.cos(alpha[i][0])
            Jl_v[i][1] = np.sin(theta3[i][0]) * np.cos(theta2[i][0] + q[i][0]) * np.cos(alpha[i][0]) + np.cos(theta3[i][0]) * np.sin(alpha[i][0])
            Jl_v[i][2] = -np.sin(theta3[i][0]) * np.sin(theta2[i][0] + q[i][0]) 

        Ja_v = np.zeros((3,3))
        for i in range(3):
            Ja_v[i][i] = self.rf * np.sin(theta2[i][0]) * np.sin(theta3[i][0])

        return Jl_v, Ja_v
    
    def check_singularity(Ja_v):
        if np.abs(np.linalg.det(Ja_v)) < 3.5*10**(-13): 
            status = True
        else: status = False
        return status
    
    def calcAngleYZ(self, x0, y0, z0):
        theta = 0
        if z0 == 0: theta, False
        y1 = -0.5 * 0.57735 * self.f # f/2 * tg 30
        y0 -= 0.5 * 0.57735 * self.e    # shift center to edge
        # z = a + b*y
        a = (x0*x0 + y0*y0 + z0*z0 +self.rf*self.rf - self.re*self.re - y1*y1)/(2*z0)
        b = (y1-y0)/z0
        # discriminant
        d = -(a+b*y1)*(a+b*y1)+self.rf*(b*b*self.rf+self.rf)
        if (d < 0): return theta, False # non-existing point
        yj = (y1 - a*b - np.sqrt(d))/(b*b + 1) # choosing outer point
        zj = a + b*yj
        theta = np.arctan2(-zj, (y1 - yj))
        return theta, True
    
    def find_theta(self, pos):
        
        A_x1 = 0
        A_y1 = -self.f*np.tan(np.deg2rad(30))/2 
        A_z1 = 0

        A_x2 = self.f*np.tan(np.deg2rad(30))/2 * np.cos(np.deg2rad(30))
        A_y2 = self.f*np.tan(np.deg2rad(30))/2 * np.sin(np.deg2rad(30))
        A_z2 = 0

        A_x3 = -self.f*np.tan(np.deg2rad(30))/2 * np.cos(np.deg2rad(30))
        A_y3 = self.f*np.tan(np.deg2rad(30))/2 * np.sin(np.deg2rad(30))
        A_z3 = 0
        # print("a_xyz",0,A_y1,A_z1,A_x2,A_y2,A_z2,A_x3,A_y3,A_z3)
        C_x1 = pos[0][0]
        C_y1 = pos[1][0] - self.e*np.tan(np.deg2rad(30))/2
        C_z1 = pos[2][0]

        C_x2 = pos[0][0] + self.e*np.tan(np.deg2rad(30))/2 * np.cos(np.deg2rad(30))
        C_y2 = pos[1][0] + self.e*np.tan(np.deg2rad(30))/2 * np.sin(np.deg2rad(30))
        C_z2 = pos[2][0] 

        C_x3 = pos[0][0] - self.e*np.tan(np.deg2rad(30))/2 * np.cos(np.deg2rad(30))
        C_y3 = pos[1][0] + self.e*np.tan(np.deg2rad(30))/2 * np.sin(np.deg2rad(30))
        C_z3 = pos[2][0]
        # print("a_xyz",C_x1,C_y1,C_z1,C_x2,C_y2,C_z2,C_x3,C_y3,C_z3)
        AC1 = np.sqrt((A_x1-C_x1)**2 + (A_y1-C_y1)**2 + (A_z1-C_z1)**2)
        AC2 = np.sqrt((A_x2-C_x2)**2 + (A_y2-C_y2)**2 + (A_z2-C_z2)**2)
        AC3 = np.sqrt((A_x3-C_x3)**2 + (A_y3-C_y3)**2 + (A_z3-C_z3)**2)

        theta2 = np.array([[0, 0, 0]]).T.astype(float)
        theta2[0][0] = np.arccos((AC1**2 - self.rf**2 - self.re**2)/(2*self.rf*self.re))
        theta2[1][0] = np.arccos((AC2**2 - self.rf**2 - self.re**2)/(2*self.rf*self.re))
        theta2[2][0] = np.arccos((AC3**2 - self.rf**2 - self.re**2)/(2*self.rf*self.re))

        theta3 = np.array([[0, 0, 0]]).T.astype(float)
        theta3[0][0] = np.pi - np.arcsin(C_x1/self.re)
        theta3[1][0] = np.pi - np.arcsin(C_x2/self.re)
        theta3[2][0] = np.pi - np.arcsin(C_x3/self.re)

        return theta2, theta3

