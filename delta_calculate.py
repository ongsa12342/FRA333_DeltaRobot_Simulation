import numpy as np


e = 44.95*2*np.sqrt(3) #155.71136760044206     #end effector #115.0
f = 200*2*np.sqrt(3)   #346.41016151377545  #base #457.3
re = 800.0 #232.0
rf = 235.0 #112.0
 
def delta_calcForward(theta1, theta2, theta3, e = e, f = f  , re = re , rf = rf ):
    #trigonometric constants
    sqrt3 = np.sqrt(3)
    pi = np.pi    #PI
    sin120 = sqrt3/2.0   
    cos120 = -0.5        
    tan60 = sqrt3
    sin30 = 0.5
    tan30 = 1/sqrt3

    t = (f-e)*tan30/2
    # t = 200
    #dtr = pi/180.0

    theta1 = np.radians(theta1)
    theta2 = np.radians(theta2)
    theta3 = np.radians(theta3)

    y1 = -(f*tan30/2 + rf*np.cos(theta1))
    z1 = -rf*np.sin(theta1)

    y2 = (f*tan30/2 + rf*np.cos(theta2))*sin30
    x2 = y2*tan60
    z2 = -rf*np.sin(theta2)

    y3 = (f*tan30/2 + rf*np.cos(theta3))*sin30
    x3 = -y3*tan60
    z3 = -rf*np.sin(theta3)

    y1_e = -(t + rf*np.cos(theta1))
    z1_e = -rf*np.sin(theta1)

    y2_e = (t + rf*np.cos(theta2))*sin30
    x2_e = y2_e*tan60
    z2_e = -rf*np.sin(theta2)

    y3_e = (t + rf*np.cos(theta3))*sin30
    x3_e = -y3_e*tan60
    z3_e = -rf*np.sin(theta3)

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
    if (d < 0): return -1 #// non-existing point

    z0 = -0.5*(b+np.sqrt(d))/a
    x0 = (a1*z0 + b1)/dnm
    y0 = (a2*z0 + b2)/dnm
    
    return [0, y1, z1], [x2, y2, z2], [x3, y3, z3] , [x0,y0,z0]