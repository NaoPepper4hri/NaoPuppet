import copy
from math import acos, asin, atan2, degrees, pi, radians, sqrt 
from numpy import dot, cross
from numpy.linalg.linalg import norm
from common import robotMirror


radian_2 = radians(2)
radian_17 = radians(17)
radian_75 = radians(75)
radian_88 = radians(88)
radian_119 = radians(100)
negative_Zaxis = [0, 0, -1]
negative_Xaxis = [-1, 0, 0]
negative_Yaxis = [0, -1, 0]

def custom_calculator (arm_vec, left_arm=True):
    arm_angles = []
   
    #start with the shoulder
    upper_arm = arm_vec[1]
    
     #------------------------- Shoulder Pitch ----------------------------------
    # To calculate the shoulder pitch (rotation of upper arm around side axis)
    #project arm on to the yz plane of kinect by making the x component 0
    yz_upper_arm = copy.deepcopy(upper_arm)
    yz_upper_arm[0] = 0.0
    # normalise the projected vector
    yz_len = norm(yz_upper_arm)
    yz_upper_arm = yz_upper_arm/yz_len
    # find the rotation by taking the cos inverse of the dot product 
    #of the projected arm with negative z of kinect
    arm_angles.append(acos(dot(yz_upper_arm, negative_Zaxis)))
    
    if dot(yz_upper_arm, negative_Yaxis) < 0:
        # negative angle
        arm_angles[-1] = -1.0*arm_angles[-1]
    #adjust for robot limits
    if arm_angles[-1] > radian_119:
        arm_angles[-1] = radian_119
    elif arm_angles[-1] < -radian_119:
        arm_angles[-1] = -radian_119
    
    #------------------------- Shoulder Roll ----------------------------------
    # To calculate the shoulder roll (rotation of upper arm around up axis)
    # we have to get the angle of rotation from the upper arm projection on the
    # yz plane with the arm vector. The pitch + roll will have to achieve the arm vector
    
    xz_upper_arm = copy.deepcopy(upper_arm)
    # normalise the projected vector
    xz_len = norm(xz_upper_arm)
    roll_vector = xz_upper_arm/xz_len
    against_vector = yz_upper_arm
    
    # find the rotation by taking the cos inverse of the dot product 
    
    arm_angles.append(acos(dot(roll_vector, against_vector)))
    if robotMirror:
        # negative angle
        arm_angles[-1] = -1.0*arm_angles[-1]
    
    if dot(roll_vector, negative_Xaxis) < 0:
        # negative angle
        arm_angles[-1] = -1.0*arm_angles[-1]
        
    # adjust for robot limits on each arm
    if left_arm:
        if arm_angles[-1] > radian_75:
            arm_angles[-1] = radian_75
        elif arm_angles[-1] < -radian_17:
            arm_angles[-1] = -radian_17
    else:
        if arm_angles[-1] > radian_17:
            arm_angles[-1] = radian_17
        elif arm_angles[-1] < -radian_75:
            arm_angles[-1] = -radian_75              
    
    #------------------------- Elbow Roll --------------------------------------
    # Calculate the eblow roll as the angle between the upper and lower amr
    upper_arm_len = norm(upper_arm)
    upper_arm = upper_arm/upper_arm_len
    lower_arm = arm_vec[2]
    lower_arm_len = norm(lower_arm)
    lower_arm = lower_arm/lower_arm_len
    
    yaw_factor = norm(cross(lower_arm, upper_arm))
    
    arm_angles.append(radian_119 * yaw_factor)
     # adjust for left right arm
    if left_arm:
        arm_angles[-1] = -1.0*arm_angles[-1]
    arm_angles.append(acos(dot(lower_arm, upper_arm)))
    # adjust for robot limits
    if arm_angles[-1] < radian_2:
        arm_angles[-1] = radian_2
    elif arm_angles[-1] > radian_88:
        arm_angles[-1] = radian_88
            
    
    # adjust for left right arm
    if left_arm:
        arm_angles[-1] = -1.0*arm_angles[-1]
        
    
    return arm_angles

calculate_arm_angles = custom_calculator