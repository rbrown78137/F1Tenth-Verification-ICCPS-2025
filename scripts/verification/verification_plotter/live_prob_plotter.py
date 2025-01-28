import matplotlib.animation as animation
from matplotlib.patches import Polygon
import rospy
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
import numpy as np
import math
from ackermann_msgs.msg import AckermannDriveStamped
import time
import signal
from threading import Thread
import pickle
from sensor_msgs.msg import Image
import cv2 as cv
import cv_bridge
import copy
import torch
import rospkg
import verification.verification_node.calculations as calculations
import verification.verification_node.verification_constants as constants
import pypoman
from matplotlib.patches import Polygon
from numpy import array, dot, hstack
from pylab import axis, gca
from scipy.spatial import ConvexHull
from nav_msgs.msg import Odometry
prob_data = {}
start_time = None
time_factor = 0.2
prob_data_display_length = 80
running_frame_count = 0
FPS = 10
last_bbox_image = None
video_writer = cv.VideoWriter('/home/ryan/Paper_Video/probability.avi', 
                         cv.VideoWriter_fourcc(*'MJPG'),
                         10, (480,480))

# Record Time
# writer = animation.FFMpegWriter(fps=10)
nav_drive_topic = "/vesc/low_level/ackermann_cmd_mux/output"
# nav_drive_topic = "/vesc/high_level/ackermann_cmd_mux/input/nav_0"
pose_topic = "/pose_data"
odom_topic = "/vesc/odom"

last_control_data = None
speed_history = []
angle_history = []
last_pose_data = None
global_start_time = 0
fig = plt.figure(figsize=(6,6))
ax1 = fig.add_subplot(1,1,1)
ani = None
colors = ["blue","red","green","yellow","purple"]
# colors.reverse()
def sigint_handler(arg1,arg2):
    rospy.signal_shutdown(reason="Program Terminated")
    global video_writer
    video_writer.release()
    exit(0)

def contraint_plotter():
    global last_pose_data, last_control_data,global_start_time
    while last_pose_data is None or last_control_data is None:
        pass
    global_start_time = time.time()
    while True:
        if len(last_pose_data) >0:
            pose_data = np.array_split(last_pose_data, len(last_pose_data)/8)
            control_data = copy.deepcopy(last_control_data)
            for pose in pose_data:
                prob_stars = calculations.probability_collision_next_n_steps(5,0,constants.dt,pose,control_data,constants.velocity_estimate)
                start_time = time.time()
                for index in range(5):
                    if not ((index+1) in prob_data):
                        prob_data[(index+1)] = []
                    prob_data[index+1].append((start_time-global_start_time,prob_stars[index]))
                time.sleep(0.01)
        else:
            
            start_time = time.time()
            for index in range(5):
                if not ((index+1) in prob_data):
                    prob_data[(index+1)] = []
                prob_data[index+1].append((start_time-global_start_time,0))
            time.sleep(0.01)

            

# def control_output_callback(navigation_topic):
#     global last_control_data
#     control_data = [navigation_topic.drive.speed, navigation_topic.drive.steering_angle]
#     last_control_data = control_data
#     # print("Saved Control Data")
#     rospy.sleep(0.001)
#     print(last_control_data)

def pose_callback(pose_data):
    global last_pose_data
    last_pose_data = pose_data.data
    # print("Saved Pose Data")
    rospy.sleep(0.001)

def animate(i):
    # print("Got to animation")
    global video_writer, FPS, running_frame_count, start_time, prob_data,ax1
    current_time = time.time()
    if start_time is None and len(prob_data.keys())>0:
        start_time = current_time
    prob_at_time = copy.deepcopy(prob_data)
    ax1.cla()
    keys = [key for key in prob_at_time]
    keys.sort()
    ax1.set_ylim([0,1])
    ax1.set_ylabel("Probability of Collision")
    ax1.set_xlabel("Time (S)")
    ax1.set_title("Probability of Collision in Next 5 Time Steps")
    for key in keys:
        one_time_step_data =prob_at_time[key]
        x = [item[0]*time_factor for item in one_time_step_data]
        y = [item[1] for item in one_time_step_data]
        if len(x)>prob_data_display_length:
            x = x[-prob_data_display_length -1:-1]
            y = y[-prob_data_display_length -1:-1]
        y = y
        color_to_display = "black"
        if int(key)-1 < len(colors):
            color_to_display = colors[int(key)-1]
        ax1.plot(x,y,color=color_to_display,label=str(key)+" future time steps")
    ax1.legend(loc="upper left")
    img = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8, sep='')
    img = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))
    img = cv.resize(img,(480,480))
    img = cv.cvtColor(img,cv.COLOR_RGB2BGR)
    if len(prob_data.keys())>0:
        if not (start_time is None) and not (current_time is None):
            while (current_time-start_time) / (1/FPS) > running_frame_count:
                running_frame_count+=1
                video_writer.write(img)
                # print(running_frame_count)
def odom_callback(odom_data):
    global last_control_data,speed_history,angle_history
    speed = odom_data.twist.twist.linear.x
    rotation = odom_data.twist.twist.angular.z + constants.camera_rotation
    if rotation <-0.8:
        rotation = -0.8
    if rotation > 0.8:
        rotation = 0.8
    # # Code For Normal VESC DATR
    # speed_history.append(speed)
    # angle_history.append(rotation)
    # if len(speed_history) > 15:
    #     speed_history.pop(0)
    # angle_history.append(rotation)
    # if len(angle_history) > 15:
    #     angle_history.pop(0)
    # average_speed = max(speed_history)
    # average_angle = rotation

    # Code for delayed vesc data
    speed_history.append(speed)
    if len(speed_history) > 30:
        speed_history.pop(0)
    angle_history.append(rotation)
    if len(angle_history) > 30:
        angle_history.pop(0)
    average_speed = speed_history[0]*4/5
    average_angle = angle_history[0]


    control_data = [average_speed, -average_angle]
    last_control_data = control_data
    # print("Saved Control Data")
    rospy.sleep(0.001)
    print(last_control_data)


if __name__ == '__main__':
    signal.signal(signal.SIGINT,sigint_handler)
    rospy.init_node('constraint_plotter')
    rospy.Subscriber(pose_topic, Float64MultiArray, pose_callback,queue_size=100)
    # rospy.Subscriber(nav_drive_topic,AckermannDriveStamped, control_output_callback, queue_size=1)
    rospy.Subscriber(odom_topic,Odometry,odom_callback,queue_size=1)
    constraint_display = Thread(target=contraint_plotter)
    constraint_display.start()
    # print("Starting Subscribers.")
    	
    # 180 = 18 seconds
    ani = animation.FuncAnimation(fig, animate, interval=100,save_count=4000)
    # ani.save("/home/ryan/Paper_Video/probability.mp4",writer)
    plt.show()
    rospy.spin()