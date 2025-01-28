#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
import verification.collision_verification.collision_probability as collision_probability
import verification.collision_verification.collision_verification_constants as constants
import verification.collision_verification.transform_perception_input as transform_perception_input
import verification.collision_verification.transform_odometry_input as transform_odometry_input
from verification.collision_verification.fast_pool import FastPool
from ackermann_msgs.msg import AckermannDriveStamped
import time
import signal
from nav_msgs.msg import Odometry
import threading
import numpy as np
import copy
import verification.collision_verification.initial_state as initial_state

# ROS TOPICS
pose_topic = "/pose_data"
nav_drive_topic = "/vesc/low_level/ackermann_cmd_mux/output" # Real Car Topic
# nav_drive_topic = "/vesc/high_level/ackermann_cmd_mux/input/nav_0" # Simulator Topic
pose_topic = "/pose_data"
odom_topic = "/vesc/odom"
publish_collision_topic = "/prob_collision"

# Global Variables to hold sensor data between threads and different time steps
sensor_global_variable_lock = threading.Lock()
recieved_control_from_sensor = None
pose_history = []
actuation_history = []
pose_time_history = []
last_logged_time = 0

# Mutiprocessing
multiprocessing_cores = 16
fast_pool = FastPool(multiprocessing_cores)

def pose_callback(pose_data):
    MAX_TIME_TO_TRACK_POSE = 2
    global recieved_control_from_sensor,sensor_global_variable_lock,pose_history,actuation_history,pose_time_history,last_logged_time
    sensor_pose_time = 0
    sensor_pose_data  = []
    if len(pose_data.data) > 1:
        sensor_pose_time = pose_data.data[0]
        sensor_pose_data = pose_data.data[1:9]
    car_reference_frame_pose_data = transform_perception_input.translate_pose_data(sensor_pose_data)
    sensor_global_variable_lock.acquire()
    if(len(sensor_pose_data) == 0):
        pose_history.clear()
        actuation_history.clear()
        pose_time_history.clear()
    else:
        while(len(pose_history)>0 and abs(pose_time_history[len(pose_time_history)-1]-sensor_pose_time)>MAX_TIME_TO_TRACK_POSE):
            pose_time_history.pop()
            pose_history.pop()
            actuation_history.pop()
        pose_history.insert(0,car_reference_frame_pose_data)
        actuation_history.insert(0,recieved_control_from_sensor)
        pose_time_history.insert(0,sensor_pose_time)
    last_logged_time = sensor_pose_time
    sensor_global_variable_lock.release()
    # print(f"Pose Data Array: {pose_data.data}")
    

def sigint_handler(arg1,arg2):
    global fast_pool
    fast_pool.shutdown()
    rospy.signal_shutdown(reason="Program Terminated")
    exit(0)

def odom_callback(odom_data):
    speed,rotation = transform_odometry_input.transform_odometry_data(odom_data)
    global recieved_control_from_sensor,sensor_global_variable_lock
    sensor_global_variable_lock.acquire()
    recieved_control_from_sensor = [speed,rotation]
    sensor_global_variable_lock.release()
    # print(f"Recieved Speed: {speed} Yaw: {rotation}")

def process_reachability_step():
    global sensor_global_variable_lock, fast_pool,pose_history,actuation_history,pose_time_history
    sensor_global_variable_lock.acquire()
    copy_pose_history = copy.deepcopy(pose_history)
    copy_actuation_history = copy.deepcopy(actuation_history)
    copy_pose_time_history = copy.deepcopy(pose_time_history)
    sensor_global_variable_lock.release()
    start_time = time.time()
    # print(f"Processing Reachability Step:{current_pose_from_perception, current_control_from_sensor}")
    if len(copy_pose_history)>0 and len(copy_pose_time_history)>0:
        # TEMP DEBUG LINES VV
        float_formatter = "{:.2f}".format
        np.set_printoptions(formatter={'float_kind':float_formatter})
        # print(f"Inputs: Pose history: {pose_history} Actuation History: {actuation_history} Pose dt history: {pose_dt_history}")
        # TEMP DEBUG LINES ^^

        # Initial state seperated from probstar calculation due to gradient and hessian being incalculable in python multiprocessing library
        X_0,sigma_0,U_0 = initial_state.initial_state(copy_pose_history,copy_actuation_history,copy_pose_time_history)
        inputs = [[1,k,constants.REACHABILITY_DT,constants.MODEL_SUBTIME_STEPS,X_0,sigma_0,U_0] for k in range(constants.K_STEPS)] 
        probstar_calculation_start_time = time.time()
        probabilities_of_collision  = fast_pool.map(collision_probability.multi_core_future_collision_probabilites, inputs)
        probstar_calculation_end_time = time.time()    
        if constants.LOG_TIMING_INFO:
            print(f"Probstar Calculation Time: {probstar_calculation_end_time-probstar_calculation_start_time}")
            print(f"Total Time: {probstar_calculation_end_time-start_time}")                 
        # inputs = [[1,k,constants.REACHABILITY_DT,constants.MODEL_SUBTIME_STEPS,pose_history,actuation_history,pose_dt_history] for k in range(constants.K_STEPS)]
        # probabilities_of_collision  = fast_pool.map(collision_probability.single_thread_future_collision_probabilites, inputs) 
        float_formatter = "{:.2f}".format
        # np.set_printoptions(formatter={'float_kind':float_formatter})
        # print(f"Probs: {np.transpose(np.array(probabilities_of_collision))} V_pi:{U_0[0]:.3f} V_omega:V_pi:{U_0[2]:.3f} X:{X_0[4]:.3f} X:{X_0[5]:.3f} T:{copy_pose_time_history[0]:.3f}")

if __name__ == '__main__':
    signal.signal(signal.SIGINT,sigint_handler)
    rospy.init_node('collision_verification_node')
    publisher = rospy.Publisher(publish_collision_topic, Float64MultiArray, queue_size=10,)
    rospy.Subscriber(pose_topic, Float64MultiArray, pose_callback,queue_size=1000)
    rospy.Subscriber(odom_topic,Odometry,odom_callback,queue_size=1000)
    while(True):
        time.sleep(0.1)
        process_reachability_step()

    