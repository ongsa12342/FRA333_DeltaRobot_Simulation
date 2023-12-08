import numpy as np

e = 44.95*2*np.sqrt(3)/1000 #155.71136760044206     #end effector #115.0
f = 200*2*np.sqrt(3)/1000   #346.41016151377545  #base #457.3
re = 800.0/1000 #232.0
rf = 235.0/1000 #112.0

m = [5, 3 ,1]
q = np.array([0, 30, 60]).reshape(3, 1)
qd = np.array([0, 10, 20]).reshape(3, 1)
qdd = np.array([0, 0, 0]).reshape(3, 1)

# constant
r = (f-e)/(2*np.sqrt(3))
alpha = [0 - 90, 120 - 90, 240 - 90]

def delta_dynamic(T, Fext, dt):
    global qd, q

    p = np.array([0, 1, 2]).reshape(3, 1) # x, y, z
    pdd = np.array([0, 0, 0]).reshape(3, 1) # xdd, ydd, zdd

    qdii = ((np.pi / 180) ** 2) * np.multiply(qd, qd)
    qdij = ((np.pi / 180) ** 2) * np.array([qd[0] * qd[1],
                                            qd[0] * qd[2],
                                            qd[1] * qd[2]]).reshape(3, 1)

    M = np.array([m[0] + (3 * m[2] / 2), 0, 0,
                  0, m[0] + (3 * m[2] / 2), 0,
                  0, 0, m[0] + (3 * m[2] / 2)]).reshape(3, 3)
    I = rf * np.array([(m[1] / 3) + (m[2] / 2), 0, 0,
                    0, (m[1] / 3) + (m[2] / 2), 0,
                    0, 0, (m[1] / 3) + (m[2] / 2)]).reshape(3, 3)
    J = np.array([0, 0, 0,
                  0, 0, 0,
                  0, 0, 0]).reshape(3, 3) # wait
    D = np.array([0, 0, 0,
                0, 0, 0,
                0, 0, 0]).reshape(3, 3) # wait
    E = np.array([0, 0, 0,
                  0, 0, 0,
                  0, 0, 0]).reshape(3, 3) # wait
    g = np.array([0, 0, -9.81]).reshape(3, 1)
    v = np.array([0, 0, 0]).reshape(3, 1)

    a1 = [0, 0, 0]
    a2 = [0, 0, 0]
    a3 = [0, 0, 0]
    Ki = [0, 0, 0]
    for i in range(3):
        a1[i] = p[0] + (r * np.cos(np.deg2rad(alpha[i]))) - (rf * np.cos(np.deg2rad(q[i])) * np.cos(np.deg2rad(alpha[i])))
        a2[i] = p[1] + (r * np.sin(np.deg2rad(alpha[i]))) - (rf * np.cos(np.deg2rad(q[i])) * np.sin(np.deg2rad(alpha[i])))
        a3[i] = p[2] - (rf * np.sin(np.deg2rad(q[i])))
        v[i] = (m[1] + m[2]) * 9.81 * rf * np.cos(np.deg2rad(q[i])) / 2
        Ki[i] = ((((p[0] * np.cos(np.deg2rad(alpha[i]))) + (p[1] * np.sin(np.deg2rad(alpha[i]))) + r) * np.sin(np.deg2rad(q[i]))) - (p[2] * np.cos(np.deg2rad(q[i]))))[0]
    A = np.array([a1[0], a1[1], a1[2],
                  a2[0], a2[1], a2[2],
                  a3[0], a3[1], a3[2]]).reshape(3, 3)
    K = np.array([Ki[0], 0, 0,
                  0, Ki[1], 0,
                  0, 0, Ki[2]]).reshape(3, 3)

    qdd = np.linalg.inv(I - (K@np.linalg.inv(A)@M@J))@(T - v + (K@np.linalg.inv(A)@((M@(g - (D@qdii) - (E@qdij))) - Fext)))
    qdd = qdd * 180 / np.pi
    
    qd = qd + (qdd * dt)
    q = q + (qd * dt)

    print(q)
    


T = np.array([0, 0, 0]).reshape(3, 1)
Fext = np.array([0, 0, 0]).reshape(3, 1)
delta_dynamic(T, Fext, 0.01)