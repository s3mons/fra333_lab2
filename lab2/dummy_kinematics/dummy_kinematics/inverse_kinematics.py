import numpy as np
import math

def inverse_kinematics(p,gamma):
    # setup parameters
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
    c_3 = round(c_3,13)
    s_3 = g2*((1-c_3**2)**0.5)
    q_3 = np.arctan2(s_3,c_3)

    #Q2
    c_2 = (l_2 + l_3*c_3)*(g1*((x**2+y**2)**0.5)-l_1) + (l_3*s_3)*(z-h)
    s_2 = (-l_3*s_3)*(g1*((x**2+y**2)**0.5)-l_1) + (l_2 + l_3*c_2)*(z-h)
    q_2 = np.arctan2(s_2,c_2)
    # print(c_1)
    # print(s_1)
    # print(c_2)
    # print(s_2)
    # print(c_3)
    # print(s_3)

    # print('q_1 = ' + str(q_1))
    # print('q_2 = ' + str(q_2))
    # print('q_3 = ' + str(q_3))


    return [q_1,q_2,q_3]
print(inverse_kinematics([0.6,0,0.3],[1,1]))

# def invki(gamma1,gamma2,x,y,z):
#     q = [0.,0.,0.]
#     q[0] = math.atan2(y/gamma1,x/gamma1) 
#     h = 0.3
#     l1 = 0.16
#     l2 = 0.35
#     l3 = 0.16
#     H = z - h
#     r = x*np.cos(-q[0])-y*np.sin(-q[0])+ - l1
#     c3 = (r**2 + H**2 - l2**2 - l3**2)/(2*l2*l3)
#     c3 = round(c3,13)
#     s3 = gamma2 * np.sqrt((1-(c3**2)))
#     q[1] = math.atan2(H,r) - math.atan2(l3*s3,l2+l3*c3)
#     q[2] = math.atan2(s3,c3)
#     return q
# print(invki(1,1,0.67,0,0.3))