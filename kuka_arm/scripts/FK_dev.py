import numpy as np
from numpy import array
from sympy import symbols, cos, sin, pi, simplify, atan2, sqrt, trigsimp, expand_trig
from sympy.matrices import Matrix

theta1,theta2,theta3,theta4,theta5,theta6,theta7 = symbols('theta1:8')
d1,d2,d3,d4,d5,d6,d7 = symbols('d1:8')
a0,a1,a2,a3,a4,a5,a6 = symbols('a0:7')
alpha0,alpha1,alpha2,alpha3,alpha4,alpha5,alpha6 = symbols('alpha0:7')

# DH parameters
#   symbol dictionary
s = {alpha0:     0, a0:      0, d1:  0.75,
     alpha1: -pi/2, a1:   0.35, d2:     0, theta2: theta2-pi/2,
     alpha2:     0, a2:   1.25, d3:     0,
     alpha3: -pi/2, a3: -0.054, d4:  1.50,
     alpha4:  pi/2, a4:      0, d5:     0,
     alpha5: -pi/2, a5:      0, d6:     0,
     alpha6:     0, a6:      0, d7: 0.303, theta7: 0}

T0_1 = Matrix([[cos(theta1), -sin(theta1), 0, a0],
               [sin(theta1) * cos(alpha0), cos(theta1) * cos(alpha0), -sin(alpha0), -sin(alpha0) * d1],
               [sin(theta1) * sin(alpha0), cos(theta1) * sin(alpha0), cos(alpha0), cos(alpha0) * d1],
               [0, 0, 0, 1]])
T0_1 = T0_1.subs(s)
R0_1 = T0_1[0:3,0:3]

T1_2 = Matrix([[cos(theta2), -sin(theta2), 0, a1],
               [sin(theta2) * cos(alpha1), cos(theta2) * cos(alpha1), -sin(alpha1), -sin(alpha1) * d2],
               [sin(theta2) * sin(alpha1), cos(theta2) * sin(alpha1), cos(alpha1), cos(alpha1) * d2],
               [0, 0, 0, 1]])
T1_2 = T1_2.subs(s)
R1_2 = T1_2[0:3,0:3]

T2_3 = Matrix([[cos(theta3), -sin(theta3), 0, a2],
               [sin(theta3) * cos(alpha2), cos(theta3) * cos(alpha2), -sin(alpha2), -sin(alpha2) * d3],
               [sin(theta3) * sin(alpha2), cos(theta3) * sin(alpha2), cos(alpha2), cos(alpha2) * d3],
               [0, 0, 0, 1]])
T2_3 = T2_3.subs(s)
R2_3 = T2_3[0:3,0:3]

T3_4 = Matrix([[cos(theta4), -sin(theta4), 0, a3],
               [sin(theta4) * cos(alpha3), cos(theta4) * cos(alpha3), -sin(alpha3), -sin(alpha3) * d4],
               [sin(theta4) * sin(alpha3), cos(theta4) * sin(alpha3), cos(alpha3), cos(alpha3) * d4],
               [0, 0, 0, 1]])
T3_4 = T3_4.subs(s)
R3_4 = T3_4[0:3,0:3]

T4_5 = Matrix([[cos(theta5), -sin(theta5), 0, a4],
               [sin(theta5) * cos(alpha4), cos(theta5) * cos(alpha4), -sin(alpha4), -sin(alpha4) * d5],
               [sin(theta5) * sin(alpha4), cos(theta5) * sin(alpha4), cos(alpha4), cos(alpha4) * d5],
               [0, 0, 0, 1]])
T4_5 = T4_5.subs(s)
R4_5 = T4_5[0:3,0:3]

T5_6 = Matrix([[cos(theta6), -sin(theta6), 0, a5],
               [sin(theta6) * cos(alpha5), cos(theta6) * cos(alpha5), -sin(alpha5), -sin(alpha5) * d6],
               [sin(theta6) * sin(alpha5), cos(theta6) * sin(alpha5), cos(alpha5), cos(alpha5) * d6],
               [0, 0, 0, 1]])
T5_6 = T5_6.subs(s)
R5_6 = T5_6[0:3,0:3]

R0_6 = R0_1*R1_2*R2_3*R3_4*R4_5*R5_6

T6_G = Matrix([[cos(theta7), -sin(theta7), 0, a6],
               [sin(theta7) * cos(alpha6), cos(theta7) * cos(alpha6), -sin(alpha6), -sin(alpha6) * d7],
               [sin(theta7) * sin(alpha6), cos(theta7) * sin(alpha6), cos(alpha6), cos(alpha6) * d7],
               [0, 0, 0, 1]])
T6_G = T6_G.subs(s)

# simplify hangs so remove it...
T0_2 = (T0_1 * T1_2)  # base to link 2
T0_3 = (T0_2 * T2_3)  # base to link 3
T0_4 = (T0_3 * T3_4)  # base to link 4
T0_5 = (T0_4 * T4_5)  # base to link 5
T0_6 = (T0_5 * T5_6)  # base to link 6
T0_G = (T0_6 * T6_G)  # base to gripper before urdf/dh adjustment

# rotate 180 degrees about Z and -90 degrees about Y
Rz = Matrix([[cos(pi), -sin(pi), 0, 0],
             [sin(pi), cos(pi), 0, 0],
             [0, 0, 1, 0],
             [0, 0, 0, 1]])

Ry = Matrix([[cos(-pi/2), 0, sin(-pi/2), 0],
             [0, 1, 0, 0],
             [-sin(-pi/2), 0, cos(-pi/2), 0],
             [0, 0, 0, 1]])
Rcorr = (Rz * Ry)
T_all = (T0_G * Rcorr)

#print('eval T0_1 = {}'.format(T0_1.evalf(subs={theta1:0.09})))
#print ('eval T_all = {}'.format(T_all.evalf(subs={})))

# center link 2 position
pos = Matrix([[0.0],[0.0],[0.0],[1.0]])
#print('pos = {}'.format(pos))

r11 = T_all[0,0]
r21 = T_all[1,0]
r31 = T_all[2,0]
r32 = T_all[2,1]
r33 = T_all[2,2]

ts = {theta1:0.72,theta2:0.88,theta3:-1.12,theta4:0,theta5:0,theta6:2.01,theta7:0}

beta = atan2(-r31, sqrt(r11**2 + r21**2)).evalf(subs=ts)  # Ry(beta)
gamma = atan2(r32,r33).evalf(subs=ts)  # Rx(gamma)
alpha = atan2(r21,r11).evalf(subs=ts)  # Rz(alpha)

print('RPY = {} {} {}'.format(gamma,beta,alpha))

# NOTE d4 is measured to link 5 (wrist center)
t = T_all * pos
print('translation from pos to gripper: {}'.format(t.evalf(subs=ts)))

# IK test

# Obtain the position ONLY from the T_all matrix
# Subsequently derive the extrinsic rotation composition matrix below to obtain orientation
px = T_all[0,3].evalf(subs=ts)
py = T_all[1,3].evalf(subs=ts)
pz = T_all[2,3].evalf(subs=ts)

#nx = T_all[0,2].evalf(subs=ts)
#ny = T_all[1,2].evalf(subs=ts)
#nz = T_all[2,2].evalf(subs=ts)
#
#print('T_all nx {} ny {} nz {}'.format(nx,ny,nz))

l = 0.303
d6 = 0

#wx = px - ((d6 + l) * nx)
#wy = py - ((d6 + l) * ny)
#wz = pz - ((d6 + l) * nz)
#
#print('computed wx = {} wy = {} wz = {}'.format(wx,wy,wz))

# link 5 translation from ROS
wx = 2.098
wy = 1.833
wz = 1.847

print('hard coded wx = {} wy = {} wz = {}'.format(wx,wy,wz))

theta_1 = np.arctan2(wy,wx)
print('theta_1 = {}'.format(theta_1))

j2x = s[a1] * np.cos(theta_1)
j2y = s[a1] * np.sin(theta_1)
j2z = s[d1]

dx = wx - j2x
dy = wy - j2y
dz = wz - j2z

A = np.sqrt(s[a3]**2 + s[d4]**2)
B = np.sqrt(dx**2 + dy**2 + dz**2)
C = s[a2]
v = (A**2 + C**2 - B**2) / (2.0*A*C)
print('v = {}'.format(v))
b = np.arccos(v)
theta_3 = np.pi/2 - b


print('j2x = {} j2y = {} j2z = {}'.format(j2x,j2y,j2z))
print('A = {} B = {} C = {}'.format(A,B,C))
print('b = {} theta_3 = {}'.format(b,theta_3))

v = (B**2 + C**2 - A**2) / (2.0*B*C)
a = np.arccos(v)

wc_a = np.arctan2(dy,dz)
print('a = {} wc_a = {} theta_2 = ?'.format(a,wc_a))

dxy = np.sqrt(dx**2 + dy**2)
angle = np.arctan2(dz,dxy)
print('angle to wc {}'.format(angle))
theta_2 = np.pi/2. - angle - a
print('theta_2 = {}'.format(theta_2))

Rx_roll = Matrix([[1,0,0],
                  [0,cos(gamma),-sin(gamma)],
                  [0,sin(gamma),cos(gamma)]])
Ry_pitch = Matrix([[cos(beta),0,sin(beta)],
                   [0,1,0],
                   [-sin(beta),0,cos(beta)]])
Rz_yaw = Matrix([[cos(alpha),-sin(alpha),0],
                 [sin(alpha),cos(alpha),0],
                 [0,0,1]])
Rz = Matrix([[cos(pi), -sin(pi), 0],
             [sin(pi), cos(pi), 0],
             [0, 0, 1]])

Ry = Matrix([[cos(-pi/2), 0, sin(-pi/2)],
             [0, 1, 0],
             [-sin(-pi/2), 0, cos(-pi/2)]])
R_corr = (Rz * Ry)
Rrpy = Rz_yaw * Ry_pitch * Rx_roll * R_corr

nx = Rrpy[0,2]
ny = Rrpy[1,2]
nz = Rrpy[2,2]

print('Rrpy nx {} ny {} nz {}'.format(nx,ny,nz))

# check equivalence
#print('T_all = {}'.format(T_all))
print('Rrpy = {}'.format(Rrpy))

wx = px - ((d6 + l) * nx)
wy = py - ((d6 + l) * ny)
wz = pz - ((d6 + l) * nz)

print('computed wx = {} wy = {} wz = {}'.format(wx,wy,wz))

print('R0_6 = {}'.format(R0_6.evalf(subs=ts)))

R0_3 = R0_1*R1_2*R2_3
R0_3 = R0_3.evalf(subs=ts)
print('R0_3 = {}'.format(R0_3))

R3_6 = R3_4*R4_5*R5_6
print('R3_6 symbolic = {}'.format(R3_6))
R3_6 = R0_3.inv('LU')*Rrpy
print('R3_6 computed = {}'.format(R3_6.evalf(subs=ts)))



