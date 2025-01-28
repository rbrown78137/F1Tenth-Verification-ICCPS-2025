import torch
import rospkg
import rospy
import signal
import matplotlib.pyplot as plt
import cv_bridge
from std_msgs.msg import Float64MultiArray
from ackermann_msgs.msg import AckermannDriveStamped
import matplotlib.animation as animation
from matplotlib.gridspec import GridSpec
import cv2 as cv
from sensor_msgs.msg import Image
import math
import numpy as np
import time
import verification.object_tracking_node.constants as otn_constants
import verification.object_tracking_node.utils as utils
import verification.object_tracking_node.image_processing as image_processing
import copy
import verification.verification_node.calculations as calculations
import verification.verification_node.verification_constants as vn_constants
import pypoman
from matplotlib.patches import Polygon
from scipy.spatial import ConvexHull
from pylab import axis, gca
import pylab
from threading import Thread

# GLOBAL VARIABLES TO EDIT
video_path = '/home/ryan/Paper_Video/20230426_004623.mp4'
# MATPlOT STUFF
fig = plt.figure()
gs = GridSpec(3, 4, figure=fig)# ,wspace=0, hspace=0 to remove spacing
ax1_video = fig.add_subplot(gs[0:2,:])
ax1_bounding_box = fig.add_subplot(gs[2,0])
ax1_bounding_box.axis("off")
ax1_prob_collision = fig.add_subplot(gs[2,1])
ax1_constraint = fig.add_subplot(gs[2,2])
ax1_semantic_segmentation = fig.add_subplot(gs[2,3])
ax1_semantic_segmentation.axis("off")
ax1_video.axis("off")

colors = ["blue","red","green","yellow","purple"]

# TOPICS
image_topic = "/camera/color/image_raw"
nav_drive_topic = "/vesc/low_level/ackermann_cmd_mux/output"
pose_topic = "/pose_data"

# PYTORCH STUFF
device = "cuda" if torch.cuda.is_available() else "cpu"
rospack = rospkg.RosPack()
yolo_model = torch.jit.load(rospack.get_path('f1tenth_verification') + "/models/object_detection/yolo.pt")
yolo_model.to(device)
pose_model = torch.jit.load(rospack.get_path('f1tenth_verification') + "/models/object_detection/pose.pt")
pose_model.to(device)
semantic_segmentation_model = torch.jit.load(rospack.get_path('f1tenth_verification') + "/models/object_detection/semantic_segmentation.pt")
semantic_segmentation_model.to(device)

# OPENCV STUFF
bridge = cv_bridge.CvBridge()
cap = cv.VideoCapture(video_path)

# GLOBAL VARIABLES
last_cv_image = None
last_pose_data = None
last_control_data = None
last_bbox_image = None
last_semantic_segmentation_image = None
last_video_image = None
prob_data = {}
global_start_time = time.time()
last_set_of_polygons= []
def sigint_handler(arg1,arg2):
    rospy.signal_shutdown(reason="Program Terminated")
    exit(0)

def animate(i):
    plt.show(False)
    print("animating")
    # Video
    global last_video_image,last_bbox_image,last_semantic_segmentation_image, last_set_of_polygons
    if not last_video_image is None:
        ax1_video.imshow(last_video_image)
        cv.waitKey(1)
        ax1_video.set_title("System View")
    
    # Semantic Segmentation 
    if i % 4 == 0:
        result = last_semantic_segmentation_image
        if not result is None:
            result = np.stack((result,)*3, axis=-1)
            ax1_semantic_segmentation.imshow(result)
            cv.waitKey(1)
        ax1_semantic_segmentation.set_title("Semantic Segmentation Image")
    
    # Bounding Box
    if i % 4 == 2:
        if not last_bbox_image is None:
            ax1_bounding_box.imshow(last_bbox_image)
            cv.waitKey(1)
        ax1_bounding_box.set_title("Bounding Box Image")
    
    #Probability Plotter
    prob_at_time = copy.deepcopy(prob_data)
    ax1_prob_collision.cla()
    keys = [key for key in prob_at_time]
    keys.sort()
    ax1_prob_collision.set_title("Probability of Collision in Next 5 Time Steps")
    for key in keys:
        one_time_step_data =prob_at_time[key]
        x = [item[0] for item in one_time_step_data]
        y = [item[1] for item in one_time_step_data]
        if len(x)>50:
            x = x[-51:-1]
            y = y[-51:-1]
        color_to_display = "black"
        if int(key)-1 < len(colors):
            color_to_display = colors[int(key)-1]
        ax1_prob_collision.plot(x,y,color=color_to_display,label=str(key)+" future time steps")
    ax1_prob_collision.legend(loc="upper left")
    
    # Constraint
    ax1_constraint.cla()
    ax1_constraint.set_xlim([-1,1])
    ax1_constraint.set_ylim([-1,1])
    polygon_copy = copy.deepcopy(last_set_of_polygons)
    if len(polygon_copy)>0:
        for index,polygon in enumerate(polygon_copy):
            plot_polygon(polygon,color=colors[index])
    ax1_constraint.set_title("Reachable Set Projected on Coordinate Directions of Car Omega")
    
    # Final Steps
    plt.show()

def plot_polygon(points, alpha=.4, color='g', linestyle='solid', fill=True,
                 linewidth=None):
    if type(points) is list:
        points = np.array(points)
    hull = ConvexHull(points)
    points = points[hull.vertices, :]
    xmin1, xmax1, ymin1, ymax1 = pylab.axis()
    xmin2, ymin2 = 1.5 * points.min(axis=0)
    xmax2, ymax2 = 1.5 * points.max(axis=0)
    ax1_constraint.axis((min(xmin1, xmin2), max(xmax1, xmax2),
          min(ymin1, ymin2), max(ymax1, ymax2)))
    patch = Polygon(
        points, alpha=alpha, color=color, linestyle=linestyle, fill=fill,
        linewidth=linewidth)
    ax1_constraint.add_patch(patch)
    

def image_callback(data):
    global last_cv_image
    last_cv_image = bridge.imgmsg_to_cv2(data,desired_encoding='passthrough')

def semantic_segmentation_image():
    global last_cv_image
    with torch.no_grad():
        width_of_network = 256
        height_of_network = 256
        rgb_image = np.copy(last_cv_image)
        resized_image = cv.resize(rgb_image,(width_of_network,height_of_network))
        input_to_network = torch.from_numpy(resized_image).permute((2,0,1)).unsqueeze(0)
        input_to_network = input_to_network.type(torch.FloatTensor).to(device)
        guess = semantic_segmentation_model(input_to_network).argmax(1).to("cpu")
        output_image = guess.mul(80).permute((1,2,0)).squeeze().type(torch.ByteTensor).numpy()
        return output_image
    
def bounding_box_image():
    global last_cv_image      

    with torch.no_grad():
        cv_image = np.copy(last_cv_image)

        if cv_image.shape[0]!= otn_constants.camera_pixel_height or cv_image.shape[1]!=otn_constants.camera_pixel_width:
            row_lower = int(cv_image.shape[0]/2 - otn_constants.camera_pixel_height / 2)
            row_upper = int(cv_image.shape[0]/2 + otn_constants.camera_pixel_height / 2)
            col_lower = int(cv_image.shape[1]/2 - otn_constants.camera_pixel_width / 2)
            col_upper = int(cv_image.shape[1]/2 + otn_constants.camera_pixel_width / 2)
            cv_image = cv_image[row_lower:row_upper,col_lower:col_upper,...]
        # For YOLO
        resized_yolo_image = cv.resize(cv_image,(otn_constants.yolo_width_of_image,otn_constants.yolo_height_of_image))
        yolo_tensor = torch.from_numpy(resized_yolo_image).to(device).unsqueeze(0).permute(0,3,1,2).to(torch.float).mul(1/256)
        network_prediction = yolo_model(yolo_tensor)
        bounding_boxes = utils.get_bounding_boxes_for_prediction(network_prediction)

        for bounding_box in bounding_boxes:
            min_x = max(0,bounding_box[1] - bounding_box[3] / 2)
            min_y = max(0,bounding_box[2] - bounding_box[4] / 2)
            max_x = min(1,bounding_box[1] + bounding_box[3] / 2)
            max_y = min(1,bounding_box[2] + bounding_box[4] / 2)
            # Debug Line
            top_left = (int(min_x * otn_constants.camera_pixel_width),int(min_y * otn_constants.camera_pixel_height))
            bottom_right = (int(max_x * otn_constants.camera_pixel_width),int(max_y * otn_constants.camera_pixel_height))
            cv.rectangle(cv_image,top_left,bottom_right,(0,255,0),3)
        return cv_image
    
def control_output_callback(navigation_topic):
    global last_control_data
    control_data = [navigation_topic.drive.speed, navigation_topic.drive.steering_angle]
    last_control_data = control_data
    # print("Saved Control Data")
    rospy.sleep(0.01)

def pose_callback(pose_data):
    global last_pose_data
    if len(pose_data.data)>0:
        last_pose_data = pose_data.data
    # print("Saved Pose Data")
    rospy.sleep(0.01)

def contraint_plotter():
    global last_pose_data, last_control_data,last_set_of_polygons
    while last_pose_data is None or last_control_data is None:
        pass
    while True:
        if len(last_pose_data) >0:
            pose_data = np.array_split(last_pose_data, len(last_pose_data)/8)
            for pose in pose_data:
                prob_stars = calculations.probability_collision_next_n_steps(5,0,vn_constants.dt,pose,last_control_data,vn_constants.velocity_estimate)
                start_time = time.time()
                for index in range(5):
                    if not ((index+1) in prob_data):
                        prob_data[(index+1)] = []
                    prob_data[index+1].append((start_time-global_start_time,prob_stars[index]))
                prob_stars = calculations.prob_star_next_time_steps(5,vn_constants.dt,pose,last_control_data,vn_constants.velocity_estimate)
                new_set_of_polygons = []
                if len(prob_stars)>0:
                    for star in prob_stars:
                        I = star
                        lb = I.pred_lb
                        ub = I.pred_ub
                        A = np.eye(I.dim)
                        C1 = np.vstack((A, -A))
                        d1 = np.concatenate([ub, -lb])
                        C = np.vstack((I.C, C1))
                        d = np.concatenate([I.d, d1])

                        c = I.V[6:8,0]
                        V = I.V[6:8,1:I.nVars+1]

                        proj = (V, c)
                        ineq = (C, d)
                        verts = pypoman.projection.project_polytope(proj, ineq,backend="ppl", base_ring="QQ")
                        if(len(verts)>0):
                            new_set_of_polygons.append(verts)
                            # print("Adding Verts")
                last_set_of_polygons = new_set_of_polygons
        else:
            
            start_time = time.time()
            for index in range(5):
                if not ((index+1) in prob_data):
                    prob_data[(index+1)] = []
                prob_data[index+1].append((start_time-global_start_time,0))
            time.sleep(0.01)
def bbox_generator():
    global last_bbox_image,last_cv_image
    while(True):
        if not last_cv_image is None:
            last_bbox_image = bounding_box_image()
        time.sleep(0.01)
        
def semantic_segmentation_generator():
    global last_semantic_segmentation_image,last_cv_image
    while(True):
        if not last_cv_image is None:
            last_semantic_segmentation_image = semantic_segmentation_image()
        time.sleep(0.01)

def video_image():
    global cap
    while(True):
        if cap.isOpened():
            ret, frame = cap.read()
            if(ret):
                rgb_image_video = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
                last_video_image = rgb_image_video
            time.sleep(1/60.0)

if __name__ == '__main__':
    signal.signal(signal.SIGINT,sigint_handler)
    rospy.init_node('constraint_plotter')
    rospy.Subscriber(image_topic, Image, image_callback,queue_size=1)
    rospy.Subscriber(pose_topic, Float64MultiArray, pose_callback,queue_size=100)
    rospy.Subscriber(nav_drive_topic,AckermannDriveStamped, control_output_callback, queue_size=1)
    constraint_display = Thread(target=contraint_plotter)
    constraint_display.start()
    semantic_segmentation_display = Thread(target=semantic_segmentation_generator)
    semantic_segmentation_display.start()
    bbox_display = Thread(target=bbox_generator)
    bbox_display.start()
    video_display = Thread(target=video_image)
    video_display.start()
    # writer = animation.FFMpegWriter(fps=10, metadata=dict(artist='Me'), bitrate=1800)
    # # 180 = 18 seconds
    ani = animation.FuncAnimation(fig, animate, interval=100)
    # ani.save("/home/ryan/Videos/test.mp4",writer)
    plt.show()
    rospy.spin()

#!/usr/bin/env python
from cv2 import COLOR_RGBA2RGB
import rospy
from sensor_msgs.msg import Image
import cv2 as cv
import torch
import cv_bridge
import math
import numpy as np
import pathlib
import time
import rospkg

device = "cuda" if torch.cuda.is_available() else "cpu"

bridge = cv_bridge.CvBridge()
image_pub = rospy.Publisher("test_bounding_boxes",Image)
