import numpy as np
import math
def forward_kinematics(q):
    joint_config = q
    n_joint = 3
    DH_pam = [[0 ,0 ,0.3 ,0],
              [0.16 ,math.pi/2 ,0 ,0],
              [0.35 ,0 ,0 ,0]]

    H = np.identity(4)

    H_end_eff = [[0 , 0 ,1 ,0.16],
                 [1 , 0 ,0 ,0],
                 [0 ,-1 ,0 ,0],
                 [0 , 0 ,0 ,1]]

    for i in range(n_joint):
        rad = joint_config[i]
        c ,s = np.cos(rad), np.sin(rad)
        c_x ,s_x = np.cos(DH_pam[i][1]), np.sin(DH_pam[i][1])
        c_z ,s_z = np.cos(DH_pam[i][3]), np.sin(DH_pam[i][3])

        T_x =  [[1 ,0 ,0 ,DH_pam[i][0]],               
                [0 ,1 ,0 ,0],
                [0 ,0, 1 ,0],
                [0 ,0 ,0 ,1]]

        R_x =  [[1 ,   0 ,   0 ,0],           
                [0 , c_x ,-s_x ,0],
                [0 , s_x , c_x ,0],
                [0 ,   0 ,   0 ,1]]

        T_z =  [[1 ,0 ,0 ,0],
                [0 ,1 ,0 ,0],
                [0 ,0 ,1 ,DH_pam[i][2]],
                [0 ,0 ,0 ,1]]

        R_z =  [[c_z ,-s_z ,0 ,0],
                [s_z , c_z ,0 ,0],
                [0   ,   0 ,1 ,0],
                [0   ,   0 ,0 ,1]]

        H_j =  [[c ,-s ,0 ,0],
                [s , c ,0 ,0],
                [0 , 0 ,1 ,0],
                [0 , 0 ,0 ,1]]

        H = H @ T_x @ R_x @ T_z @ R_z @ H_j      

    H = H@H_end_eff

    return H
    
# print(forward_kinematics([math.pi,0,0]))

