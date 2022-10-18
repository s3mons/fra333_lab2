import numpy as np
import math

def inverse_kinematics(p,gamma):
    # setup parameters
    flag = True
    x = p[0]
    y = p[1]
    z = p[2]
    g1 = gamma[0]
    g2 = gamma[1]
    

    h = 0.3
    l_1 = 0.16
    l_2 = 0.35
    l_3 = 0.16

    # Q1
    s_1 = y/g1
    c_1 = x/g1
    q_1 = np.arctan2(s_1,c_1)

    # Q3
    c_3 = ((g1*((x**2+y**2)**0.5) - l_1)**2 + (z-h)**2 - l_2**2 - l_3**2)/(2*l_2*l_3)
    # c_3 = round(c_3,13)
    if -1 <= c_3 and c_3 <=1 :
        s_3 = g2*((1-c_3**2)**0.5)
        q_3 = np.arctan2(s_3,c_3)
    else:
        flag = False
        return [0. ,0. ,0.],flag
        
    #Q2
    c_2 = (l_2 + l_3*c_3)*(g1*((x**2+y**2)**0.5)-l_1) + (l_3*s_3)*(z-h)
    s_2 = (-l_3*s_3)*(g1*((x**2+y**2)**0.5)-l_1) + (l_2 + l_3*c_3)*(z-h)
    q_2 = np.arctan2(s_2,c_2)


    return [q_1,q_2,q_3],flag

# print(inverse_kinematics([0.6,0,0.3],[1,1]))
# print(inverse_kinematics([0.3,0,0.3],[-1,1]))
