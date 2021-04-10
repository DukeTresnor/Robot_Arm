# This should calculate the velocity kinematics of a 3R robot arm with 3 degrees of freedom
# I think we have a 4th little bit representing the end effector too.

# Imports
import numpy as np
import importlib
import exp_coordinates as exp_c
import physical_properties as pp

# Importing exp_coordinates and physical properties -- not necessary formatting?
# exp_coordinates = importlib.import_module('exp_coordinates')
# physical_properties = importlib.import_module('physical_properties')


# Array of joint angles -- in the future this will be calling joint angles from another file probably?
angle_position = np.array([
    [0],
    [0],
    [0],
    [0]
])

theta_1 = angle_position[0][0]
theta_2 = angle_position[1][0]
theta_3 = angle_position[2][0]
theta_4 = angle_position[3][0]

# Array of joint velocities -- in the future this will be calling joint velocities from another file
angle_velocity = np.array([
    [1.0],
    [1.0],
    [1.0],
    [1.0]
])

theta_vel_1 = angle_velocity[0][0]
theta_vel_2 = angle_velocity[1][0]
theta_vel_3 = angle_velocity[2][0]
theta_vel_4 = angle_velocity[3][0]

'''
print(theta_1)
print('---')
print(type(theta_1))
'''

'''
Twists and Jacobian -- goal is to arrive at the space twists being equal to the space 
    Jacobian matrix times the list of angular velocities
    This is useful if the whole state is determined by the joint angles, and so the
        time derivative of the state is the vector of joint velocities.
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

# Screw axis 2
ang_vel_2 = np.array([
    [-np.sin(theta_1) ],
    [np.cos(theta_1) ],
    [1]
])

lin_vel_2 = np.array([
    [-pp.L_1*np.cos(theta_1)],
    [-pp.L_1*np.sin(theta_1)],
    [0]
])
'''
print(ang_vel_2)
print('---')
print(lin_vel_2)
print('---')
print(-pp.L_1*np.cos(theta_1))
'''
# Screw axis 3
ang_vel_3 = np.array([
    [-np.sin(theta_1)],
    [np.cos(theta_1)],
    [1]
])

lin_vel_3 = np.array([
    [-np.cos(theta_1) * (pp.L_1 + pp.L_2 * np.cos(theta_2))],
    [-np.sin(theta_1) * (pp.L_1 + pp.L_2 * np.cos(theta_2))],
    [np.sin(theta_1) * pp.L_2 * np.sin(theta_2) * (np.sin(theta_1) + np.cos(theta_2) ) ]
])


# Screw axis 4
ang_vel_4 = np.array([
    [np.cos(theta_4) * np.sin(theta_1 + theta_2)],
    [np.sin(theta_4) * np.sin(theta_1 + theta_2)],
    [np.cos(theta_1 + theta_2)]
])

lin_vel_4 = np.array([
    [pp.L_2 * np.cos(theta_1 + theta_2) * np.sin(theta_2) * np.sin(theta_1) - np.sin(theta_1) * np.sin(theta_1 + theta_2) * (pp.L_1 + pp.L_2 * np.cos(theta_2))],
    [-pp.L_2 * np.cos(theta_1 + theta_2) * np.sin(theta_2) * np.cos(theta_1) + np.cos(theta_1) * np.sin(theta_1 + theta_2) * (pp.L_1 + pp.L_2 * np.cos(theta_2))],
    [pp.L_2 * np.sin(theta_1 + theta_2) * np.sin(theta_2) * (np.sin(theta_1) * np.cos(theta_1) - np.sin(theta_1) * np.cos(theta_1))] 
]) # 3rd element here goes to 0


# Jacobians -- J_si, columns of J_s(theta), jacobian in the space frame
# use exp_transform = np.vstack((exp_transform, fourth_row))
jacb_s_1 = np.vstack((ang_vel_1, lin_vel_1))
jacb_s_2 = np.vstack((ang_vel_2, lin_vel_2))
jacb_s_3 = np.vstack((ang_vel_3, lin_vel_3))
jacb_s_4 = np.vstack((ang_vel_4, lin_vel_4))

jacb_s = np.column_stack((jacb_s_1, jacb_s_2))
jacb_s = np.column_stack((jacb_s, jacb_s_3))

# Final space Jacobian -- jacb_s -- probably could have implemented this better
jacb_s = np.column_stack((jacb_s, jacb_s_4))


'''
 structure twist = jacobian * theta_dot
 An object's twist is the concatenated angular and linear velocities of that object. 
   we want to multiply the space jacobian by the vector of joint velocities, then create
   angular 
'''
twist_space = jacb_s.dot(angle_velocity)

# Determining angular velocity from the space twist
ang_vel = exp_c.Vector(twist_space[0][0], twist_space[1][0], twist_space[2][0])

# Determining linear velocity from the space twist
lin_vel = exp_c.Vector(twist_space[3][0], twist_space[4][0], twist_space[5][0])

'''
 Printing the space twist, which has the angular and then the linear velocity of the
   end effector
'''
print('First three elements are the x, y, and z components of the angular velocity of the end effector (roll, pitch, yaw)')
print(twist_space)
print('Second three elements are the x, y, and z components of the linear velocity of the end effector')