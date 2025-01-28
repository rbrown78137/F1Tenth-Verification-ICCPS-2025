import verification.collision_verification.collision_verification_constants as constants
from math import cos,sin
'''
File to Translate real world sensor data to car center of reference frame
In our implementation, our camera was slightly loose, so the angle of rotation or camera_yaw_offset was different between different recordings
The various camera yaw offsets indicate our best estimate for how tilted our camera was on the car during each of the recordings
'''
# Offset Values Determined From F1Tenth Implementation Based on Position of Camera at Runtime
camera_x_offset = 0
camera_y_offset = -0.05
camera_yaw_offset = 0

# Recording 1, 3-5
if constants.RECORDING_NUMBER == 1 or constants.RECORDING_NUMBER == 3 or constants.RECORDING_NUMBER == 4 or constants.RECORDING_NUMBER == 5 or constants.RECORDING_NUMBER == 6:
    camera_yaw_offset = -0.02

# Recording 2
if constants.RECORDING_NUMBER == 2:
    camera_yaw_offset = -.05

# Passing Clip / Recording 6
if constants.RECORDING_NUMBER == 7:
    camera_yaw_offset = -0.025

if constants.RECORDING_NUMBER == 8:
    camera_yaw_offset = -0.04 # Crossing 2

def translate_pose_data(pose_data):
    if not(pose_data is None) and len(pose_data)>0:
        pose_x = pose_data[0]
        pose_y = pose_data[1]
        theta_i = pose_data[2]
        theta_j = pose_data[3]
        pose_x_uncertainty = pose_data[4]
        pose_y_uncertainty = pose_data[5]
        theta_i_uncertainty = pose_data[5]
        theta_j_uncertainty = pose_data[6]

        new_pose_x = (pose_x-camera_x_offset) * cos(camera_yaw_offset) - (pose_y-camera_y_offset)*sin(camera_yaw_offset)
        new_pose_y = (pose_x-camera_x_offset) * sin(camera_yaw_offset) + (pose_y-camera_y_offset)*cos(camera_yaw_offset)
        new_pose_x_uncertainty = pose_x_uncertainty * cos(camera_yaw_offset) - pose_y_uncertainty*sin(camera_yaw_offset)
        new_pose_y_uncertainty = pose_x_uncertainty * sin(camera_yaw_offset) + pose_y_uncertainty*cos(camera_yaw_offset)
        new_theta_i = theta_i * cos(camera_yaw_offset) - theta_j*sin(camera_yaw_offset)
        new_theta_j = theta_i * sin(camera_yaw_offset) + theta_j*cos(camera_yaw_offset)
        new_theta_i_uncertainty = theta_i_uncertainty * cos(camera_yaw_offset) - theta_j_uncertainty*sin(camera_yaw_offset)
        new_theta_j_uncertainty = theta_i_uncertainty * sin(camera_yaw_offset) + theta_j_uncertainty*cos(camera_yaw_offset)
        # print(f"X:{new_pose_x} Y:{new_pose_y}")
        return [new_pose_x,new_pose_y,new_theta_i,new_theta_j,new_pose_x_uncertainty,new_pose_y_uncertainty,new_theta_i_uncertainty,new_theta_j_uncertainty]
    return []