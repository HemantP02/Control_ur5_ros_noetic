#!/usr/bin/env python3

import numpy as np
import cmath

# ***** Coefficients *****
d1 =  0.1273
a2 = -0.612
a3 = -0.5723
a7 = 0.075
d4 =  0.163941
d5 =  0.1157
d6 =  0.0922

d = np.array([0.089159, 0, 0, 0.10915, 0.09465, 0.0823]) 
a = np.array([0, -0.425, -0.39225, 0, 0, 0]) 
alph = np.array([np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0])  

# ***** Forward Kinematics *****
def AH(n, th, c):
    T_a = np.identity(4)
    T_a[0,3] = a[n-1]
    T_d = np.identity(4)
    T_d[2,3] = d[n-1]

    Rzt = np.array([[np.cos(th[n-1,c]), -np.sin(th[n-1,c]), 0, 0],
                    [np.sin(th[n-1,c]), np.cos(th[n-1,c]), 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])

    Rxa = np.array([[1, 0, 0, 0],
                    [0, np.cos(alph[n-1]), -np.sin(alph[n-1]), 0],
                    [0, np.sin(alph[n-1]), np.cos(alph[n-1]), 0],
                    [0, 0, 0, 1]])

    A_i = np.dot(T_d, np.dot(Rzt, np.dot(T_a, Rxa)))

    return A_i

def HTrans(th, c):  
    A_1 = AH(1, th, c)
    A_2 = AH(2, th, c)
    A_3 = AH(3, th, c)
    A_4 = AH(4, th, c)
    A_5 = AH(5, th, c)
    A_6 = AH(6, th, c)
      
    T_06 = np.dot(A_1, np.dot(A_2, np.dot(A_3, np.dot(A_4, np.dot(A_5, A_6)))))

    return T_06

# ***** Inverse Kinematics *****
def invKine(desired_pos):
    th = np.zeros((6, 8))
    P_05 = (np.dot(desired_pos, np.array([0, 0, -d6, 1]).T) - np.array([0, 0, 0, 1 ]))
  
    # **** theta1 ****
    psi = np.arctan2(P_05[2-1], P_05[1-1])
    phi = np.arccos(d4 / np.sqrt(P_05[2-1]**2 + P_05[1-1]**2))
    th[0, 0:4] = np.pi/2 + psi + phi
    th[0, 4:8] = np.pi/2 + psi - phi
  
    # **** theta5 ****
    cl = [0, 4] # wrist up or down
    for i in range(0, len(cl)):
        c = cl[i]
        T_10 = np.linalg.inv(AH(1, th, c))
        T_16 = T_10 @ desired_pos
        th[4, c:c+2] = np.arccos((T_16[2,3]-d4)/d6)
        th[4, c+2:c+4] = -np.arccos((T_16[2,3]-d4)/d6)

    # **** theta6 ****
    cl = [0, 2, 4, 6]
    for i in range(0, len(cl)):
        c = cl[i]
        T_10 = np.linalg.inv(AH(1, th, c))
        T_16 = np.linalg.inv(T_10 @ desired_pos)
        th[5, c:c+2] = np.arctan2((-T_16[1,2]/np.sin(th[4, c])), (T_16[0,2]/np.sin(th[4, c])))

    # **** theta3 ****
    cl = [0, 2, 4, 6]
    for i in range(0, len(cl)):
        c = cl[i]
        T_10 = np.linalg.inv(AH(1, th, c))
        T_65 = AH(6, th, c)
        T_54 = AH(5, th, c)
        T_14 = (T_10 @ desired_pos) @ np.linalg.inv(T_54 @ T_65)
        P_13 = T_14 @ np.array([0, -d4, 0, 1]).T - np.array([0, 0, 0, 1]).T
        t3 = np.arccos((np.linalg.norm(P_13)**2 - a2**2 - a3**2 )/(2 * a2 * a3))
        th[2, c] = t3
        th[2, c+1] = -t3

    # **** theta2 and theta 4 ****
    cl = [0, 1, 2, 3, 4, 5, 6, 7]
    for i in range(0, len(cl)):
        c = cl[i]
        T_10 = np.linalg.inv(AH(1, th, c))
        T_65 = np.linalg.inv(AH(6, th, c))
        T_54 = np.linalg.inv(AH(5, th, c))
        T_14 = (T_10 @ desired_pos) @ T_65 @ T_54
        P_13 = T_14 @ np.array([0, -d4, 0, 1]).T - np.array([0, 0, 0, 1]).T
        th[1, c] = -np.arctan2(P_13[1], -P_13[0]) + np.arcsin(a3* np.sin(th[2,c])/np.linalg.norm(P_13))
        T_32 = np.linalg.inv(AH(3, th, c))
        T_21 = np.linalg.inv(AH(2, th, c))
        T_34 = T_32 @ T_21 @ T_14
        th[3, c] = np.arctan2(T_34[1,0], T_34[0,0])
  
    return th

def invKine_xyz(x, y, z):
    desired_pos = np.array([[1, 0, 0, x],
                            [0, 1, 0, y],
                            [0, 0, 1, z],
                            [0, 0, 0, 1]])

    return invKine(desired_pos)
def main():
    x = 0.3
    y = 0.2
    z = 0.5
    joint_angles = invKine_xyz(x, y, z)
    print("Joint angles:", joint_angles)

if __name__ == "__main__":
    main()
