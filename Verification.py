import numpy as np

def check_foward(e,f,re,rf,theta):
    np.tan(np.deg2rad(30))
    a = (f-e) * np.tan(np.deg2rad(30)) /2 + rf * np.cos(np.radians(theta))
    b = re
    c = np.sqrt(b*b - a*a) - rf * np.sin(np.radians(theta))
    return c