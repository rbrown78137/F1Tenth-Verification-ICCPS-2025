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
import verification.collision_verification.collision_predicate as collision_predicate
import verification.collision_verification.collision_verification_constants as constants
import verification.collision_verification.transform_odometry_input as transform_odometry_input
import verification.collision_verification.transform_perception_input as transform_perception_input
import verification.collision_verification.initial_state as initial_state
import pypoman
from matplotlib.patches import Polygon
from numpy import array, dot, hstack
from pylab import axis, gca
from scipy.spatial import ConvexHull
from nav_msgs.msg import Odometry
import threading
import sys
import os
import matplotlib

font = {'family' : 'normal',
        'weight' : 'bold',
        'size'   : 16}

matplotlib.rc('font', **font)

start_time = None
running_frame_count = 0
standard_deviations_of_star = 5
FPS = 10
video_writer = cv.VideoWriter('/home/ryan/Paper_Video/constraint.avi', 
                         cv.VideoWriter_fourcc(*'MJPG'),
                         10, (480,480))
# writer = animation.FFMpegWriter(fps=10)

nav_drive_topic = "/vesc/low_level/ackermann_cmd_mux/output"
# nav_drive_topic = "/vesc/high_level/ackermann_cmd_mux/input/nav_0"
pose_topic = "/pose_data"
odom_topic = "/vesc/odom"

# last_control_data = None
# speed_history = []
# angle_history = []
last_pose_data = None
last_set_of_polygons = []
last_set_of_polygons_time = 0
bridge = cv_bridge.CvBridge()

fig = plt.figure(figsize=(6,6))
ax1 = fig.add_subplot(1,1,1)
COLORS = ["blue","red","green","yellow","purple"]
COLOR_MAP = {"blue":[0,0,255/255],"red":[255/255,0,0],"green":[0,127/255,0],"yellow":[255/255,165/255,0],"purple":[127/255,0,127/255]}

# colors.reverse()

# Global Variables to hold sensor data between threads and different time steps
sensor_global_variable_lock = threading.Lock()
recieved_control_from_sensor = None
global_pose_history = []
global_actuation_history = []
global_pose_time_history = []
last_logged_time = 0

def sigint_handler(arg1,arg2):
    global video_writer
    video_writer.release()
    rospy.signal_shutdown(reason="Program Terminated")
    exit(0)

def plot_polygon(points,title, alpha=.4, color='g', linestyle='solid', fill=True,
                 linewidth=None):
    if type(points) is list:
        points = array(points)
    hull = ConvexHull(points)
    points = points[hull.vertices, :]
    # xmin1, xmax1, ymin1, ymax1 = axis()
    # xmin2, ymin2 = 1.5 * points.min(axis=0)
    # xmax2, ymax2 = 1.5 * points.max(axis=0)
    # axis((min(xmin1, xmin2), max(xmax1, xmax2),
    #       min(ymin1, ymin2), max(ymax1, ymax2)))
    
    patch = Polygon(
        points, alpha=alpha, color=color, linestyle=linestyle, fill=fill,
        linewidth=linewidth,label= title)
    ax1.add_patch(patch)
    # ax1.legend(loc="upper left",font={'family' : 'normal',
    #     'weight' : 'bold',
    #     'size'   : 16})
    
def contraint_plotter():
    # print("Got To constraints")
    global sensor_global_variable_lock,global_pose_history,global_actuation_history,global_pose_time_history,last_set_of_polygons_time,last_set_of_polygons
    while True:
        sensor_global_variable_lock.acquire()
        copy_pose_history = copy.deepcopy(global_pose_history)
        copy_actuation_history = copy.deepcopy(global_actuation_history)
        copy_pose_time_history = copy.deepcopy(global_pose_time_history)
        sensor_global_variable_lock.release()

        if len(copy_pose_history) > 2 and len(copy_pose_time_history) > 2:
            pose_data = np.array_split(copy_pose_history[0], len(copy_pose_history[0])/8)
            for pose in pose_data:
                X_0,sigma_0,U_0 = initial_state.initial_state(copy_pose_history,copy_actuation_history,copy_pose_time_history)
                # predicates = collision_predicate.predicate_next_k_timesteps(5,constants.REACHABILITY_DT,X_0,sigma_0,U_0)
                predicates = collision_predicate.predicate_next_k_timesteps(5,constants.REACHABILITY_DT,X_0,U_0)
                new_set_of_polygons = []

                for predicate in predicates:
                    C = predicate[0]
                    d = predicate[1]

                    E = np.zeros((2, 8))
                    E[0, 0] = 1.
                    E[1, 1] = 1.
                    f = np.zeros(2)
                    proj = (E, f)
                    ineq = (C, d)
                    # print("Finding Polytope")
                    sys.stdout = open(os.devnull, 'w')
                    verts = pypoman.projection.project_polyhedron(proj,ineq)[0]
                    sys.stdout = sys.__stdout__
                    if len(verts)>0:
                        first_point = verts[0]
                        different_point = False
                        for vert in verts:
                            for idx in range(len(vert)):
                                if first_point[idx] != vert[idx]:
                                    different_point = True
                        if different_point is False:
                            verts = []
                    # verts = pypoman.projection.project_polytope(proj, ineq,backend="ppl", base_ring="QQ")
                    # print("Found Polytope")
                    if(len(verts)>0):
                        new_set_of_polygons.append(verts)
                        # print("Adding Verts")
                if len(new_set_of_polygons) > 0:
                    sensor_global_variable_lock.acquire()
                    last_set_of_polygons = new_set_of_polygons
                    last_set_of_polygons_time = time.time()
                    sensor_global_variable_lock.release()
                    # print("Defining New Polygon")
                elif (last_set_of_polygons_time is not None) and time.time()-last_set_of_polygons_time >0.2:
                    sensor_global_variable_lock.acquire()
                    last_set_of_polygons = new_set_of_polygons
                    sensor_global_variable_lock.release()
                    # print("Empty Polygon")
        elif len(copy_pose_history) == 0:
            # if time.time()-last_set_of_polygons_time >0.2:
            sensor_global_variable_lock.acquire()
            last_set_of_polygons = []
            sensor_global_variable_lock.release()
        time.sleep(0.1)

def odom_callback(odom_data):
    speed,rotation = transform_odometry_input.transform_odometry_data(odom_data)
    global recieved_control_from_sensor,sensor_global_variable_lock
    sensor_global_variable_lock.acquire()
    recieved_control_from_sensor = [speed,rotation]
    sensor_global_variable_lock.release()
    # print(f"Recieved Speed: {speed} Yaw: {rotation}")

def pose_callback(pose_data):
    MAX_TIME_TO_TRACK_POSE = 2
    global recieved_control_from_sensor,sensor_global_variable_lock,global_pose_history,global_actuation_history,global_pose_time_history,last_logged_time
    sensor_pose_time = 0
    sensor_pose_data  = []
    if len(pose_data.data) > 1:
        sensor_pose_time = pose_data.data[0]
        sensor_pose_data = pose_data.data[1:9]
    car_reference_frame_pose_data = transform_perception_input.translate_pose_data(sensor_pose_data)
    if not(recieved_control_from_sensor == None):
        sensor_global_variable_lock.acquire()
        if(len(sensor_pose_data) == 0):
            global_pose_history.clear()
            global_actuation_history.clear()
            global_pose_time_history.clear()
        else:
            if len(global_pose_history)>0 and len(global_pose_history[0]) == 0:
                global_pose_history.clear()
                global_pose_time_history.clear()
                global_actuation_history.clear()
            while(len(global_pose_history)>0 and abs(global_pose_time_history[len(global_pose_time_history)-1]-sensor_pose_time)>MAX_TIME_TO_TRACK_POSE):
                global_pose_time_history.pop()
                global_pose_history.pop()
                global_actuation_history.pop()
            global_pose_history.insert(0,car_reference_frame_pose_data)
            global_actuation_history.insert(0,recieved_control_from_sensor)
            global_pose_time_history.insert(0,sensor_pose_time)
        last_logged_time = sensor_pose_time
        sensor_global_variable_lock.release()
    # print(f"Pose Data Array: {pose_data.data}")
    

def animate(i):
    global video_writer, FPS, running_frame_count, start_time,last_pose_data,last_set_of_polygons,ax1,sensor_global_variable_lock,global_pose_history
    current_time = time.time()
    if start_time is None and len(global_pose_history)>2:
        start_time = current_time
    # print("Got to animation")
    ax1.cla()
    ax1.set_xlim([-1.4,1])
    ax1.set_ylim([-1,1])
    ax1.set_xlabel("$X_{\omega} - X_{\pi}$ (meters)",fontdict={'family' : 'normal',
        'weight' : 'bold',
        'size'   : 16})
    ax1.set_ylabel("$Y_{\omega} - Y_{\pi}$ (meters)",fontdict={'family' : 'normal',
        'weight' : 'bold',
        'size'   : 16})
    sensor_global_variable_lock.acquire()
    polygon_copy = copy.deepcopy(last_set_of_polygons)
    sensor_global_variable_lock.release()
    # print("Got To Animation")
    if len(polygon_copy)>0:
        # print("Found Non EMPTY POLYGON") 
        # ax1.set_xlim([0,0])
        # ax1.set_ylim([0,0])
        for index,polygon in enumerate(polygon_copy):
            plot_polygon(polygon,color=COLOR_MAP[COLORS[index]],title=f"{index+1} Future Time Steps")
    # else:
    #     ax1.set_xlim([0,0])
    #     ax1.set_ylim([0,0])
    # print("Finished Plotting")
    ax1.set_title("Collision Bounding Polytope",fontdict={'family' : 'normal',
        'weight' : 'bold',
        'size'   : 17})
    plt.margins(x=0)
    fig.subplots_adjust(
        top=0.95,
        bottom=0.10,
        left=0.20,
        right=0.95,
        hspace=0.2,
        wspace=0.2
    )
    img = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8, sep='')
    img = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))
    img = cv.resize(img,(480,480))
    img = cv.cvtColor(img,cv.COLOR_RGB2BGR)
    if not global_pose_history is None:
        if not (start_time is None) and not (current_time is None):
            while (current_time-start_time) / (1/FPS) > running_frame_count:
                running_frame_count+=1
                video_writer.write(img)
                # print(running_frame_count)
# def live_prob_plotter():
#     global fig
#     ani = animation.FuncAnimation(fig, animate, interval=1000)

if __name__ == '__main__':
    signal.signal(signal.SIGINT,sigint_handler)
    rospy.init_node('constraint_plotter')
    rospy.Subscriber(pose_topic, Float64MultiArray, pose_callback,queue_size=100)
    rospy.Subscriber(odom_topic,Odometry,odom_callback,queue_size=1)
    constraint_display = Thread(target=contraint_plotter)
    constraint_display.start()
    # print("Starting Subscribers.")
    ani = animation.FuncAnimation(fig, animate, interval=10,save_count=4000)
    # ani.save("/home/ryan/Paper_Video/constraint.mp4",writer)
    plt.show()
    # print("Test")
    rospy.spin()
    