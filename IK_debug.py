from sympy import *
from time import time
from mpmath import radians
import tf
import numpy as np

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[],
              5:[]}

# some values out of matrix inverse are very slightly out of range for sin/cos (e.g. 1.00001172958) leading to nan
def limit(sin_cos_val):
    if sin_cos_val < -1.0:
        return -1.0
    elif sin_cos_val > 1.0:
        return 1.0
    return sin_cos_val


def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)
    start_time = time()
    
    ########################################################################################
    ## 

    ## Insert IK code here!

    px = req.poses[x].position.x
    py = req.poses[x].position.y
    pz = req.poses[x].position.z

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
        [req.poses[x].orientation.x, req.poses[x].orientation.y,
            req.poses[x].orientation.z, req.poses[x].orientation.w])

    Rx_roll = Matrix([[1.,0.,0.],
                      [0.,np.cos(roll),-np.sin(roll)],
                      [0.,np.sin(roll),np.cos(roll)]])

    Ry_pitch = Matrix([[np.cos(pitch),0.,np.sin(pitch)],
                       [0.,1.,0.],
                       [-np.sin(pitch),0.,np.cos(pitch)]])

    Rz_yaw = Matrix([[np.cos(yaw),-np.sin(yaw),0.],
                     [np.sin(yaw),np.cos(yaw),0.],
                     [0.,0.,1.]])

    Rz = Matrix([[np.cos(np.pi), -np.sin(np.pi), 0.],
                 [np.sin(np.pi), np.cos(np.pi), 0.],
                 [0., 0., 1.]])

    Ry = Matrix([[np.cos(-np.pi/2.), 0., np.sin(-np.pi/2.)],
                 [0., 1., 0.],
                 [-np.sin(-np.pi/2.), 0., np.cos(-np.pi/2.)]])
    R_corr = (Rz * Ry)
    Rrpy = (Rz_yaw * Ry_pitch * Rx_roll * R_corr).evalf()


    theta1,theta2,theta3,theta4,theta5,theta6,theta7 = symbols('theta1:8')
    d1,d2,d3,d4,d5,d6,d7 = symbols('d1:8')
    a0,a1,a2,a3,a4,a5,a6 = symbols('a0:7')
    alpha0,alpha1,alpha2,alpha3,alpha4,alpha5,alpha6 = symbols('alpha0:7')

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

    T1_2 = Matrix([[cos(theta2), -sin(theta2), 0, a1],
                   [sin(theta2) * cos(alpha1), cos(theta2) * cos(alpha1), -sin(alpha1), -sin(alpha1) * d2],
                   [sin(theta2) * sin(alpha1), cos(theta2) * sin(alpha1), cos(alpha1), cos(alpha1) * d2],
                   [0, 0, 0, 1]])
    T1_2 = T1_2.subs(s)

    T2_3 = Matrix([[cos(theta3), -sin(theta3), 0, a2],
                   [sin(theta3) * cos(alpha2), cos(theta3) * cos(alpha2), -sin(alpha2), -sin(alpha2) * d3],
                   [sin(theta3) * sin(alpha2), cos(theta3) * sin(alpha2), cos(alpha2), cos(alpha2) * d3],
                   [0, 0, 0, 1]])
    T2_3 = T2_3.subs(s)

    T3_4 = Matrix([[cos(theta4), -sin(theta4), 0, a3],
                   [sin(theta4) * cos(alpha3), cos(theta4) * cos(alpha3), -sin(alpha3), -sin(alpha3) * d4],
                   [sin(theta4) * sin(alpha3), cos(theta4) * sin(alpha3), cos(alpha3), cos(alpha3) * d4],
                   [0, 0, 0, 1]])
    T3_4 = T3_4.subs(s)

    T4_5 = Matrix([[cos(theta5), -sin(theta5), 0, a4],
                   [sin(theta5) * cos(alpha4), cos(theta5) * cos(alpha4), -sin(alpha4), -sin(alpha4) * d5],
                   [sin(theta5) * sin(alpha4), cos(theta5) * sin(alpha4), cos(alpha4), cos(alpha4) * d5],
                   [0, 0, 0, 1]])
    T4_5 = T4_5.subs(s)

    T5_6 = Matrix([[cos(theta6), -sin(theta6), 0, a5],
                   [sin(theta6) * cos(alpha5), cos(theta6) * cos(alpha5), -sin(alpha5), -sin(alpha5) * d6],
                   [sin(theta6) * sin(alpha5), cos(theta6) * sin(alpha5), cos(alpha5), cos(alpha5) * d6],
                   [0, 0, 0, 1]])
    T5_6 = T5_6.subs(s)

    T6_G = Matrix([[cos(theta7), -sin(theta7), 0, a6],
                   [sin(theta7) * cos(alpha6), cos(theta7) * cos(alpha6), -sin(alpha6), -sin(alpha6) * d7],
                   [sin(theta7) * sin(alpha6), cos(theta7) * sin(alpha6), cos(alpha6), cos(alpha6) * d7],
                   [0, 0, 0, 1]])
    T6_G = T6_G.subs(s)

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
    T0_2 = (T0_1 * T1_2)  # base to link 2
    T0_3 = (T0_2 * T2_3)  # base to link 3
    T0_4 = (T0_3 * T3_4)  # base to link 4
    T0_5 = (T0_4 * T4_5)  # base to link 5
    T0_6 = (T0_5 * T5_6)  # base to link 6
    T0_G = (T0_6 * T6_G)  # base to gripper before urdf/dh adjustment

    T_all = (T0_G * Rcorr)


    R0_1 = T0_1[0:3,0:3]
    R1_2 = T1_2[0:3,0:3]
    R2_3 = T2_3[0:3,0:3]
    R3_4 = T3_4[0:3,0:3]
    R4_5 = T4_5[0:3,0:3]
    R5_6 = T5_6[0:3,0:3]
    R0_3 = R0_1*R1_2*R2_3
    R3_6_sym = R3_4*R4_5*R5_6

    print('R3_6_sym = {}'.format(R3_6_sym))

    lv = 0.303
    d6v = 0.0

    nx = float(Rrpy[0,2])
    ny = float(Rrpy[1,2])
    nz = float(Rrpy[2,2])
    wx = px - ((d6v + lv) * nx)
    wy = py - ((d6v + lv) * ny)
    wz = pz - ((d6v + lv) * nz)

    theta1v = np.arctan2(wy,wx) #numeric

    print('theta1v = {}'.format(theta1v))

    j2x = s[a1] * np.cos(theta1v) #numeric
    j2y = s[a1] * np.sin(theta1v) #numeric
    j2z = s[d1]

    dx = wx - j2x #numeric
    dy = wy - j2y #numeric
    dz = wz - j2z #numeric

    A = np.sqrt(s[a3]**2 + s[d4]**2)
    B = np.sqrt(dx**2 + dy**2 + dz**2)
    C = s[a2]
    v = (A**2 + C**2 - B**2) / (2.0*A*C)
    b = np.arccos(v)
    theta3v = (np.pi/2. - b)

    print('theta3v = {}'.format(theta3v))

    v = (B**2 + C**2 - A**2) / (2.0*B*C)
    a = np.arccos(v)

    dxy = np.sqrt(dx**2 + dy**2)
    angle = np.arctan2(dz,dxy)
    theta2v = np.pi/2. - angle - a
    print('angle to wc {} theta2v = {}'.format(angle,theta2v))

    R0_3 = R0_3.evalf(subs={theta1:theta1v,theta2:theta2v,theta3:theta3v})    # still a sympy thing
    R3_6 = R0_3.inv('LU')*Rrpy
    R3_6 = R3_6.evalf(subs={theta1:theta1v,theta2:theta2v,theta3:theta3v})   # still a sympy thing
    #print('R3_6_sym = {}'.format(R3_6_sym))
    #print('R3_6 eval = {}'.format(R3_6))

    sin_theta5_cos_theta6 = float(R3_6[1,0].evalf())
    cos_theta5 = limit(float(R3_6[1,2].evalf()))
    print('cos_theta5 = {} R3_6 {}'.format(cos_theta5,R3_6))
    sin_theta4_sin_theta5 = float(R3_6[2,2].evalf())
    theta5v = np.arccos(cos_theta5)
    sin_theta5v = np.sin(theta5v)
    if abs(sin_theta5v) > 0.001:
        v = limit(sin_theta4_sin_theta5 / sin_theta5v)
        theta4v = np.arcsin(v)
        v = limit(sin_theta5_cos_theta6 / sin_theta5v)
        theta6v = np.arccos(v)
    else:
        print('sin theta 5 {} R3_6 vals {} {}'.format(sin_theta5v,sin_theta4_sin_theta5,sin_theta5_cos_theta6))
        theta4v = np.pi/4.  # any
        theta6v = np.pi/4.  # any

    ## 
    ########################################################################################
    
    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    ## (OPTIONAL) YOUR CODE HERE!
    ts = {theta1: theta1v, theta2: theta2v, theta3: theta3v, theta4: theta4v, theta5: theta5v, theta6: theta6v, theta7: 0.}
    px = T_all[0,3].evalf(subs=ts)
    py = T_all[1,3].evalf(subs=ts)
    pz = T_all[2,3].evalf(subs=ts)


    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_wc = [wx,wy,wz] # <--- Load your calculated WC values in this array
    your_ee = [px,py,pz] # <--- Load your calculated end effector value from your forward kinematics
    ########################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1v-test_case[2][0])
    t_2_e = abs(theta2v-test_case[2][1])
    t_3_e = abs(theta3v-test_case[2][2])
    t_4_e = abs(theta4v-test_case[2][3])
    t_5_e = abs(theta5v-test_case[2][4])
    t_6_e = abs(theta6v-test_case[2][5])
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple positions. It is best to add your forward kinmeatics to \
           \nconfirm whether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)




if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = 1

    test_code(test_cases[test_case_number])
