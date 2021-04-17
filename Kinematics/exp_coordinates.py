# This establishes some function that are useful for determining transformation matrices
#   and configurations of robots

# Imports
import numpy as np


# Work needs to be done to restructure this so that it implements classes better,
#   or to not define my linear and angular velocities with classes and just use arrays instead.
# Creating the vector class -- has x, y, and z components (copied from stack exchange)
class Vector(object):
    def __init__(self, x,y,z):
        self.x = x
        self.y = y
        self.z = z

    # overload []
    def __getitem__(self, index):
        data = [self.x,self.y,self.z]
        return data[index]

    # overload set []
    def __setitem__(self, key, item):
        if (key == 0):
            self.x = item
        elif (key == 1):
            self.y = item
        elif (key == 2):
            self.z = item
        #TODO: Default should throw excetion

def skew(vector):
    '''
    This function takes a vector and creates a skew-symmetric representation of it
    arguments -- vector with 3 elements
    returns -- skew-symmetric representation of the input vector (3x3 matrix)
    
    to-do: logic to catch vector inputs with wrong number of coordinates
    '''

    v1 = vector.x
    v2 = vector.y
    v3 = vector.z

    skew_vec = np.array([
        [0, -v3, v2],
        [v3, 0, -v1],
        [-v2, v1, 0]
    ])

    return skew_vec

def matrix_exp_helper(theta, ang_vel):
    '''
    Represents the power series of the matrix exponential using sines and consines 
    (from Euler's Forumla)
    arguments
        scalar value representing rotation angle
        3x1 angular velocity vector
    returns
        inifinite power series as a finite sum of sines and cosines
        should return a 3x3 matrix
    '''

    # Getting skew-symmetric representation of angular velocity vector ang_vel
    skew_ang_vel = skew(ang_vel)

    I = np.eye(3)

    ident = I * theta

    linear_skew_vel = (1 - np.cos(theta)) * skew_ang_vel

    quadratic_skew_vel = (theta - np.sin(theta)) * np.dot(skew_ang_vel, skew_ang_vel)


    G = ident + linear_skew_vel + quadratic_skew_vel

    return G

def matrix_exponential_rotation(theta, ang_vel):
    '''
    Returns the matrix exponential for a rotation about the axis ang_vel by an amount theta
        Known as the Rodrigue's formula for rotations
    arguments
        theta -- scalar value for angle of rotationta
        ang_vel -- vector for angular velocity
    returns
        matrix exponential representing the rotation that occurs about an axis ang_vel by and amount
            theta
        Should be a 3x3 matrix
    '''

    # Getting skew-symmetric representation of angular velocity ang_vel
    skew_ang_vel = skew(ang_vel)

    # Identity matrix
    I = np.eye(3)
    # Angle value multiplied by the skew-symetric representation of the angular velocity
    linear_skew_vel = np.sin(theta) * skew_ang_vel
    # Angle value multiplied by the square of the skew-symetric representation of the angular velocity
    quadratic_skew_vel = (1 - np.cos(theta)) * np.dot(skew_ang_vel, skew_ang_vel)
    # Summing each term up
    exp_rotation = I + linear_skew_vel + quadratic_skew_vel


    return exp_rotation

def matrix_exponential_transform(theta, ang_vel, lin_vel):
    '''
    Calculates your matrix exponential for a transformation occuring from a screw axis and a rotation angle
        screw axis is made from angular and linear velocities of object
    arguments
        theta -- scalar value for angle of rotation
        ang_vel -- vector for angular velocity
        lin_vel -- vector for linear velocity
    returns
        matrix exponential representing the transformation that takes object from reference
            orientation and position i into orientation and position i+1, and so on.
        Should be a 4x4 matrix
    '''

    # Creating the matrix exponential for the rotation
    exp_rotation = matrix_exponential_rotation(theta, ang_vel)

    # Calculating G(theta) -- helper function
    G = matrix_exp_helper(theta, ang_vel)

    # Calculating position vector G*v
    lin_vel = np.array([
        [lin_vel.x],
        [lin_vel.y],
        [lin_vel.z]
    ])
    #p = G * lin_vel
    p = np.dot(G, lin_vel)
    '''
    Overall matrix exponential that describes transformation
        1st -- concatinating the rotation matrix with the position vector
        2nd -- concatinating the above with the 4th row -- [0,0,0,1]
    '''
    exp_transform = np.column_stack((exp_rotation, p))
    fourth_row = np.array([
        [0, 0, 0, 1]
    ])
    exp_transform = np.vstack((exp_transform, fourth_row))
    

    return exp_transform






'''
# test junk

ang_vel = Vector(0, 1, 0)
lin_vel = Vector(-1, 0, 0)
theta = np.pi/2.0


y = matrix_exponential_transform(theta, ang_vel, lin_vel)

port = matrix_exponential_rotation(theta, ang_vel)
'''


