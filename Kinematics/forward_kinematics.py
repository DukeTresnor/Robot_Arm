# This should calculate the forward kinematics of a 3R robot arm with 3 degrees of freedom
# I think we have a 4th little bit representing the end effector too.

# Imports
import numpy as np
import importlib
import exp_coordinates as exp_c
import physical_properties as pp
from numpy.linalg import multi_dot

# Importing exp_coordinates and physical properties -- this formatting isn't necessary?
# exp_coordinates = importlib.import_module('exp_coordinates')
# physical_properties = importlib.import_module('physical_properties')


# Array of joint angles -- in the future this will be calling joint angles from another file probably?
# Samples
angle_array = np.array([
    [0],
    [0],
    [0],
    [0]
])

theta_1 = angle_array[0][0]
theta_2 = angle_array[1][0]
theta_3 = angle_array[2][0]
theta_4 = angle_array[3][0]


# End-efector position and orientation with joint angles set to 0
#   Home or zero position of robot and its end-effector
M = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, pp.L_1 + pp.L_2 + pp.L_3],
    [0, 0, 0, 1]
])


'''
# Screw axis 1
ang_vel_1 = np.array([
    [0],
    [0],
    [1]
])

lin_vel_1 = np.array([
    [0],
    [0],
    [0]
])
'''
# using class notation
ang_vel_1 = exp_c.Vector(0,0,1)
lin_vel_1 = exp_c.Vector(0,0,0)

'''
# Screw axis 2
ang_vel_2 = np.array([
    [0],
    [1],
    [0]
])

lin_vel_2 = np.array([
    [-L_1],
    [0],
    [0]
])
'''
# using class notation
ang_vel_2 = exp_c.Vector(0,1,0)
lin_vel_2 = exp_c.Vector(-pp.L_1,0,0)

'''
# Screw axis 3
ang_vel_3 = np.array([
    [0],
    [1],
    [0]
])

lin_vel_3 = np.array([
    [-(L_1 + L_2)],
    [0],
    [0]
])
'''
# using class notation
ang_vel_3 = exp_c.Vector(0,1,0)
lin_vel_3 = exp_c.Vector(-(pp.L_1 + pp.L_2),0,0)

'''
# Screw axis 4
ang_vel_4 = np.array([
    [0],
    [0],
    [1]
])

lin_vel_4 = np.array([
    [0],
    [0],
    [0]
])
'''
# using class notation
ang_vel_4 = exp_c.Vector(0,0,1)
lin_vel_4 = exp_c.Vector(0,0,0)

'''
Exponential coordinates formula to describe position of end effector
T(theta) = exp([S1]theta1) * exp([S2]theta2) * exp([S3]theta3) * exp([S4]theta4) * M
T is the transformation matrix that takes one from space reference frame towards the end-effector frame
[Si] are skew-symmetric representations of the screw axes
thetai are the joint angles of each of the joints 1 through 4
M is the end-effector frame configuration when the robot is in its zero position

'''

T_01 = exp_c.matrix_exponential_transform(theta_1, ang_vel_1, lin_vel_1)
T_12 = exp_c.matrix_exponential_transform(theta_2, ang_vel_2, lin_vel_2)
T_23 = exp_c.matrix_exponential_transform(theta_3, ang_vel_3, lin_vel_3)
T_34 = exp_c.matrix_exponential_transform(theta_4, ang_vel_4, lin_vel_4)


# Overall kinematics -- Calculates the transformation matrix that gives the position and orientation
#   of the end effector
T_main = multi_dot([T_01, T_12, T_23, T_34, M])

print('---')
print('Transformation matrix for the end-effector')
print('The nine elements in the upper left describe the orientation of the end-effector relative to the space or 0 frame')
print('The three elements in the upper right describe the position vector of the end-effector relative to the space or 0 frame')
print('Bottom row of elements are placer values to make the matrix math work properly')
print(T_main)
