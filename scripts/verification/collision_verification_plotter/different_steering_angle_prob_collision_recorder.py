import matplotlib.animation as animation
import rospy
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
import numpy as np
import time
import signal
from threading import Thread
import cv2 as cv
import copy
from nav_msgs.msg import Odometry
import threading
from verification.collision_verification.fast_pool import FastPool
import verification.collision_verification.collision_probability as collision_probability
import verification.collision_verification.collision_verification_constants as constants
import verification.collision_verification.transform_perception_input as transform_perception_input
import verification.collision_verification.transform_odometry_input as transform_odometry_input
import verification.collision_verification.initial_state as initial_state
import pickle
import matplotlib

font = {'family' : 'normal',
        'weight' : 'bold',
        'size'   : 13}

matplotlib.rc('font', **font)

# Collision Time
COLLISION_TIME = -1
if constants.RECORDING_NUMBER ==1:
    COLLISION_TIME = 1682487988.4

# Algorithm Settings
TIMESTEP = 4
steering_angle_modifications = [-0.174533, -0.1309, -0.0872665, -0.0436332, 0]
steering_angle_modification_labels = ["-10\u00b0","-7.5\u00b0", "-5\u00b0", "-2.5\u00b0", "No Change\u00b0"]
# steering_angle_modifications = [-0.174533, -0.0872665, 0, 0.0872665, 0.174533]
# steering_angle_modification_labels = ["-10\u00b0","-5\u00b0", "0\u00b0", "5\u00b0", "10\u00b0"]
# steering_angle_modifications = [-0.174533, -0.1309, -0.0872665, -0.0436332, 0]
# steering_angle_modification_labels = ["-10\u00b0","-7.5\u00b0", "-5\u00b0", "-2.5\u00b0", "0\u00b0"]
# ROS TOPICS
pose_topic = "/pose_data"
nav_drive_topic = "/vesc/low_level/ackermann_cmd_mux/output"
pose_topic = "/pose_data"
odom_topic = "/vesc/odom"
publish_collision_topic = "/prob_collision"

# Global Variables to hold sensor data between threads and different time steps
sensor_global_variable_lock = threading.Lock()
recieved_control_from_sensor = None
global_pose_history = []
global_actuation_history = []
global_pose_time_history = []
last_logged_time = 0

# Mutiprocessing
multiprocessing_cores = 16
fast_pool = FastPool(multiprocessing_cores)

# RECORDING SETTING VARIABLES
PROBABILITY_HISTORY_LENGTH = 4 # s
REACHABILITY_STEP_SLEEP_TIME = 0.1 # s
REPLAY_SPEED = 1
FPS = 10
COLORS = ["blue","red","green","yellow","purple"]
# RECORDING GLOBAL VARIABLES
global_video_writer = cv.VideoWriter('/home/ryan/Paper_Video/different_steering_angles.avi', 
                         cv.VideoWriter_fourcc(*'MJPG'),
                         10, (480,480))
global_figure = plt.figure(figsize=(6,6))
global_subplot = global_figure.add_subplot(1,1,1)
animation_event_dispatcher = None
global_frame_recording_start_time = None
global_running_frame_count = 0

# PROBABILIY OF COLLISION HISTORY
global_probability_collision_history = []

# DATA TO RECORD FOR OTHER FILES AND DENSITY PLOT RECORDER
frame_sensor_data = []
def sigint_handler(arg1,arg2):
    fast_pool.shutdown()
    rospy.signal_shutdown(reason="Program Terminated")
    global global_video_writer
    global_video_writer.release()
    with open('saved_data/frame_history.pkl', 'wb') as f:
        pickle.dump(frame_sensor_data, f)
    print("Ending Program and Saving Probability data to file.")
    exit(0)

# POSE CALLBACK TAKEN FROM collision_verification_ros_node.py
def pose_callback(pose_data):
    MAX_TIME_TO_TRACK_POSE = 2
    global recieved_control_from_sensor,sensor_global_variable_lock,global_pose_history,global_actuation_history,global_pose_time_history,last_logged_time,global_probability_collision_history
    sensor_pose_time = 0
    sensor_pose_data  = []
    if len(pose_data.data) > 1:
        sensor_pose_time = pose_data.data[0]
        sensor_pose_data = pose_data.data[1:9]
    car_reference_frame_pose_data = transform_perception_input.translate_pose_data(sensor_pose_data)
    sensor_global_variable_lock.acquire()
    if(len(sensor_pose_data) == 0):
        global_pose_history.clear()
        global_actuation_history.clear()
        global_pose_time_history.clear()
        global_probability_collision_history.clear()
    else:
        while(len(global_pose_history)>0 and abs(global_pose_time_history[len(global_pose_time_history)-1]-sensor_pose_time)>MAX_TIME_TO_TRACK_POSE):
            global_pose_time_history.pop()
            global_pose_history.pop()
            global_actuation_history.pop()
        global_pose_history.insert(0,car_reference_frame_pose_data)
        global_actuation_history.insert(0,recieved_control_from_sensor)
        global_pose_time_history.insert(0,sensor_pose_time)
    last_logged_time = sensor_pose_time
    sensor_global_variable_lock.release()
    
# ODOM CALLBACK TAKEN FROM collision_verification_ros_node.py
def odom_callback(odom_data):
    speed,rotation = transform_odometry_input.transform_odometry_data(odom_data)
    global recieved_control_from_sensor,sensor_global_variable_lock
    sensor_global_variable_lock.acquire()
    recieved_control_from_sensor = [speed,rotation]
    sensor_global_variable_lock.release()

def process_reachability_step_thread():
    while(True):
        global sensor_global_variable_lock,fast_pool,global_pose_history,global_actuation_history,global_pose_time_history,global_probability_collision_history
        sensor_global_variable_lock.acquire()
        copy_pose_history = copy.deepcopy(global_pose_history)
        copy_actuation_history = copy.deepcopy(global_actuation_history)
        copy_pose_time_history = copy.deepcopy(global_pose_time_history)
        sensor_global_variable_lock.release()

        if len(copy_pose_history)>2 and len(copy_pose_time_history)>0:
            X_0,sigma_0,U_0 = initial_state.initial_state(copy_pose_history,copy_actuation_history,copy_pose_time_history)
            inputs = []
            for idx in range(len(steering_angle_modifications)):
                modified_U_0 = copy.deepcopy(U_0)
                modified_U_0[1] = modified_U_0[1] + steering_angle_modifications[idx]
                inputs.append([1,TIMESTEP-1,constants.REACHABILITY_DT,constants.MODEL_SUBTIME_STEPS,X_0,sigma_0,modified_U_0])
            probabilities_of_collision  = fast_pool.map(collision_probability.multi_core_future_collision_probabilites, inputs)
            current_time = copy_pose_time_history[0]

            # float_formatter = "{:.2f}".format
            # np.set_printoptions(formatter={'float_kind':float_formatter})
            # print(f"Probs: {np.transpose(np.array(probabilities_of_collision))} varphi_x_omega:{X_0[6]:.3f} varphi_y_omega:{X_0[7]:.3f} T:{copy_pose_time_history[0]:.3f}")
            # print(f"Probs: {np.transpose(np.array(probabilities_of_collision))} V_pi:{U_0[0]:.3f} V_omega:{U_0[2]:.3f} X:{X_0[4]:.3f} Y:{X_0[5]:.3f} T:{copy_pose_time_history[0]:.3f}")
                
            sensor_global_variable_lock.acquire()
            global_probability_collision_history.append((current_time,probabilities_of_collision,[copy_pose_history,copy_actuation_history,copy_pose_time_history]))
            # REMOVE ALL ELEMENTS PAST HISTORY TIME LIMIT
            while (abs(global_probability_collision_history[0][0]-current_time)>PROBABILITY_HISTORY_LENGTH):
                global_probability_collision_history.pop(0)
            sensor_global_variable_lock.release()
        else:
            global_probability_collision_history.clear()
        time.sleep(REACHABILITY_STEP_SLEEP_TIME)      

def animate(i):
    # print("Got to animation")
    global global_video_writer, FPS, global_subplot,sensor_global_variable_lock, global_probability_collision_history, global_frame_recording_start_time, global_running_frame_count
    sensor_global_variable_lock.acquire()
    probability_collision_history = copy.deepcopy(global_probability_collision_history)
    sensor_global_variable_lock.release()
    global_subplot.cla()
    global_subplot.set_ylim([0,1])
    global_subplot.set_ylabel("Probability of Collision",fontdict={'family' : 'normal',
        'weight' : 'bold',
        'size'   : 15})
    global_subplot.set_xlabel("Timesteps Elapsed",fontdict={'family' : 'normal',
        'weight' : 'bold',
        'size'   : 15})
    global_subplot.set_title("Future Probabilities of Collision",fontdict={'family' : 'normal',
        'weight' : 'bold',
        'size'   : 15})
    if len(probability_collision_history)>0 and len(probability_collision_history[0])>0:
        # SET START TIME FOR FRAME RECORDING
        if global_frame_recording_start_time is None:
            global_frame_recording_start_time = time.time()
        # DISPLAY PROBABILITY OF COLLISION
        for modification_idx in range(len(probability_collision_history[0][1])):
            x = [(item[0]-probability_collision_history[0][0]) /constants.REACHABILITY_DT for item in probability_collision_history]
            y = [item[1][modification_idx] for item in probability_collision_history]

            color_to_display = COLORS[modification_idx]
            global_subplot.plot(x,y,color=color_to_display,label=steering_angle_modification_labels[modification_idx])
        if COLLISION_TIME > 0:
            global_subplot.vlines(x=[(COLLISION_TIME-probability_collision_history[0][0]) /constants.REACHABILITY_DT],ymin=0,ymax=1,color=(0,0,0),linestyles="dashed", label="Point of Collision")
    global_subplot.legend(loc="upper left", prop={'family':'normal', 'size':11})
    global_figure.subplots_adjust(
        top=0.95,
        bottom=0.11,
        left=0.15,
        right=0.95,
        hspace=0.2,
        wspace=0.2
    )
    img = np.fromstring(global_figure.canvas.tostring_rgb(), dtype=np.uint8, sep='')
    img = img.reshape(global_figure.canvas.get_width_height()[::-1] + (3,))
    img = cv.resize(img,(480,480))
    img = cv.cvtColor(img,cv.COLOR_RGB2BGR)
    
    if not (global_frame_recording_start_time is None) and len(probability_collision_history)>0:
        current_time = time.time()
        while (current_time-global_frame_recording_start_time) / (1/FPS) > global_running_frame_count:
            frame_sensor_data.append([probability_collision_history[-1][0],probability_collision_history[-1][2]])
            global_running_frame_count+=1
            # UNCOMMENT TO ACTUALLY RECORD IMAGE
            global_video_writer.write(img)
            cv.imshow("Probability of Collision", img)
            cv.waitKey(1)



if __name__ == '__main__':
    signal.signal(signal.SIGINT,sigint_handler)
    rospy.init_node('constraint_plotter')
    rospy.Subscriber(pose_topic, Float64MultiArray, pose_callback,queue_size=100)
    rospy.Subscriber(odom_topic,Odometry,odom_callback,queue_size=1)
    constraint_display = Thread(target=process_reachability_step_thread)
    constraint_display.start()
    	
    # 180 = 18 seconds
    animation_event_dispatcher = animation.FuncAnimation(global_figure, animate, interval=100,save_count=4000)
    # ani.save("/home/ryan/Paper_Video/probability.mp4",writer)
    plt.show()
    rospy.spin()