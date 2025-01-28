import copy
import verification.collision_verification.collision_verification_constants as constants

'''
This file translates the raw odometry input to the most likely true value of the speed and steering angle on the car 
The utilized odomtery sensor on the F1Tenth car simply measured the velocity output command and did not actually measure the motor speed
This file attemps to accurately mimic acceleration forces on the vehicle by avoiding instantaneous velocity chagnes
Additionally the PID controller had about a 0.6 second delay before actually sending out commands to the motor

Acceleration forces choosen for each recording are based on the actual acceleration of the car based on the acceleration calculated from VICON pose data
'''

input_delay = 0.6

max_deceleration = 3 #m/s^2
max_acceleration = 1.6 #m/s^2
velocity_multiplier = 0.9

# Actual Speed of motor reduced in configuration files between some recordings. This if statement attemps to fix this
if constants.RECORDING_NUMBER == 2:
    new_velocity_mulitplier = 0.52
    max_acceleration = max_acceleration * (velocity_multiplier/new_velocity_mulitplier) #m/s^2 
    velocity_multiplier = new_velocity_mulitplier

odometry_speed_history = []
odometry_steering_history = []
last_speed = 0
last_recieved_time = None
SECONDS_TRACK_HISTORY = 3

# Function takes in odomtery input to incorporate input delay to controller and acceleration forces between instantaneous velocity changes
def transform_odometry_data(odom_data):
    global last_recieved_time,odometry_speed_history,last_speed
    current_seconds = odom_data.header.stamp.to_sec()
    modified_base_speed = odom_data.twist.twist.linear.x
    GT_speed = odom_data.twist.twist.linear.x
    GT_STEERING = odom_data.twist.twist.angular.z

    # Clear history for recordings with loops
    if not(last_recieved_time is None) and last_recieved_time>current_seconds:
        odometry_speed_history.clear()
        odometry_steering_history.clear()

    odometry_steering_history.append([current_seconds,GT_STEERING])
    odometry_speed_history.append([current_seconds,GT_speed])
    while odometry_steering_history[0][0] < current_seconds-input_delay:
        odometry_steering_history.pop(0)
    while odometry_speed_history[0][0] < current_seconds-input_delay:
        odometry_speed_history.pop(0)
    max_speed_gain = 0
    max_speed_loss = 0
    if len(odometry_speed_history)>1:
        max_speed_gain = max_acceleration * (current_seconds-odometry_speed_history[len(odometry_speed_history)-2][0])
        max_speed_loss = max_deceleration * (current_seconds-odometry_speed_history[len(odometry_speed_history)-2][0])
    desired_speed = odometry_speed_history[0][1]
    modified_base_speed = max(last_speed-max_speed_loss,min(last_speed+max_speed_gain,desired_speed))
    modified_base_speed = max(0,modified_base_speed)
    modified_rotation = odometry_steering_history[0][1]
    # print(f"Modified SPEED: {abs(modified_base_speed):.3f} GT:{abs(GT_speed):.3f}")
    # print(f"Modified STEERING: {modified_rotation} GT:{GT_STEERING}")
    last_speed = modified_base_speed
    last_recieved_time = current_seconds
    modified_base_speed = velocity_multiplier * modified_base_speed
    return modified_base_speed, modified_rotation
    
    
        