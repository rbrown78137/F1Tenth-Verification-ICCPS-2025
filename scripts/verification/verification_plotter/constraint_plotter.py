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

last_control_data = None
speed_history = []
angle_history = []
last_pose_data = None
last_set_of_polygons = []
last_set_of_polygons_time = 0
bridge = cv_bridge.CvBridge()

fig = plt.figure(figsize=(6,6))
ax1 = fig.add_subplot(1,1,1)
colors = ["blue","red","green","yellow","purple"]
# colors.reverse()
def sigint_handler(arg1,arg2):
    rospy.signal_shutdown(reason="Program Terminated")
    global video_writer
    video_writer.release()
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
    ax1.legend(loc="upper left")
    
def contraint_plotter():
    # print("Got To constraints")
    global last_pose_data, last_control_data,last_set_of_polygons,last_set_of_polygons_time
    while last_pose_data is None or last_control_data is None:
        pass
    while True:
        if len(last_pose_data) >0:
            pose_data = np.array_split(last_pose_data, len(last_pose_data)/8)
            for pose in pose_data:
                prob_stars = calculations.prob_star_next_time_steps(5,constants.dt,pose,last_control_data,constants.velocity_estimate)
                new_set_of_polygons = []
                if len(prob_stars)>0:
                    for star in prob_stars:
                        I = star
                        lb = I.pred_lb
                        ub = I.pred_ub
                        A = np.eye(I.dim)
                        C1 = np.vstack((A, -A))
                        d1 = np.concatenate([ub*standard_deviations_of_star, -lb*standard_deviations_of_star])
                        C = np.vstack((I.C, C1))
                        d = np.concatenate([I.d, d1])

                        c = I.V[6:8,0]
                        V = I.V[6:8,1:I.nVars+1]

                        proj = (V, c)
                        ineq = (C, d)
                        # print("Finding Polytope")
                        verts = pypoman.projection.project_polyhedron(proj,ineq,canonicalize=False)[0]
                        # verts = pypoman.projection.project_polytope(proj, ineq,backend="ppl", base_ring="QQ")
                        # print("Found Polytope")
                        if(len(verts)>0):
                            new_set_of_polygons.append(verts)
                            # print("Adding Verts")
                if len(new_set_of_polygons) >0:
                    last_set_of_polygons = new_set_of_polygons
                    last_set_of_polygons_time = time.time()
                    print("Defining New Polygon")
                elif time.time()-last_set_of_polygons_time >0.2:
                    last_set_of_polygons = new_set_of_polygons
                    print("Empty Polygon")
                rospy.sleep(0.01)
        elif len(last_pose_data) == 0:
            # if time.time()-last_set_of_polygons_time >0.2:
            last_set_of_polygons = []

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

def pose_callback(pose_data):
    global last_pose_data
    last_pose_data = pose_data.data
    # print("Saved Pose Data")
    rospy.sleep(0.001)

def animate(i):
    global video_writer, FPS, running_frame_count, start_time,last_pose_data,last_control_data,last_set_of_polygons,ax1
    current_time = time.time()
    if start_time is None and not last_control_data is None:
        start_time = current_time
    # print("Got to animation")
    ax1.cla()
    ax1.set_xlim([-1,1])
    ax1.set_ylim([-1,3])
    ax1.set_xlabel("X / P Coordinate of Car Omega (meters)")
    ax1.set_ylabel("Z Coordinate of Car Omega (meters)")
    polygon_copy = copy.deepcopy(last_set_of_polygons)
    print("Got To Animation")
    if len(polygon_copy)>0:
        print("Found Non EMPTY POLYGON") 
        # ax1.set_xlim([0,0])
        # ax1.set_ylim([0,0])
        for index,polygon in enumerate(polygon_copy):
            plot_polygon(polygon,color=colors[index],title=f"{index+1} Future Time Steps")
    # else:
    #     ax1.set_xlim([0,0])
    #     ax1.set_ylim([0,0])
    print("Finished Plotting")
    ax1.set_title("Projection of Reachable Set on Coordinate Plane of Car Omega")
    img = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8, sep='')
    img = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))
    img = cv.resize(img,(480,480))
    img = cv.cvtColor(img,cv.COLOR_RGB2BGR)
    if not last_control_data is None:
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
    
    # stars = calculations.prob_star_next_time_steps(30,0.1,[0,3,0.5,0.5,1,1,1e-4,1e-4], [1,0.01],0)
    # # verts = getVertices(stars[0])
    # for star in stars:
    #     I = star
    #     lb = I.pred_lb
    #     ub = I.pred_ub
    #     A = np.eye(I.dim)
    #     C1 = np.vstack((A, -A))
    #     d1 = np.concatenate([ub, -lb])
    #     C = np.vstack((I.C, C1))
    #     d = np.concatenate([I.d, d1])

    #     c = I.V[6:8,0]
    #     V = I.V[6:8,1:I.nVars+1]

    #     proj = (V, c)
    #     ineq = (C, d)
    #     verts = pypoman.projection.project_polytope(proj, ineq,backend="ppl", base_ring="QQ")
    #     if(len(verts)>0):
    #         plot_polygon(verts)
    #         cv.waitKey(1)

    #     c = I.V[0:2,0]
    #     V = I.V[0:2,1:I.nVars+1]

    #     proj = (V, c)
    #     ineq = (C, d)
    #     # verts = pypoman.projection.project_polytope(proj, ineq)
    #     # if(len(verts)>0):
    #     #     pass
    #     #     # plot_polygon(verts)
    # test = 0