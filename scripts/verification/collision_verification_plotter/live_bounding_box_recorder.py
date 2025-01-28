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
import verification.object_tracking_node.utils as utils
import verification.object_tracking_node.constants as constants
import verification.object_tracking_node.image_processing as image_processing
import matplotlib.animation as animation

image_topic = "/camera/color/image_raw"
last_cv_image = None

device = "cuda" if torch.cuda.is_available() else "cpu"
rospack = rospkg.RosPack()

yolo_model = torch.jit.load(rospack.get_path('f1tenth_verification') + "/models/object_detection/yolo.pt")
yolo_model.to(device)

pose_model = torch.jit.load(rospack.get_path('f1tenth_verification') + "/models/object_detection/pose.pt")
pose_model.to(device)

fig = plt.figure(figsize=(6,6))
ax1 = fig.add_subplot(1,1,1)
ax1.axis("off")

bridge = cv_bridge.CvBridge()
start_time = None
running_frame_count = 0
FPS = 10
last_bbox_image = None
video_writer = cv.VideoWriter('/home/ryan/Paper_Video/test.avi', 
                         cv.VideoWriter_fourcc(*'MJPG'),
                         10, (640,480))

def sigint_handler(arg1,arg2):
    global video_writer
    video_writer.release()
    rospy.signal_shutdown(reason="Program Terminated")
    exit(0)
    
def image_callback(new_image):
    global video_writer, FPS, running_frame_count, start_time
    current_time = time.time()
    if start_time is None:
        start_time = current_time
    global last_cv_image,bridge
    cv_image = bridge.imgmsg_to_cv2(new_image,desired_encoding='rgb8')
    last_cv_image = cv.cvtColor(cv_image,cv.COLOR_BGR2RGB)
    with torch.no_grad():
        cv_image = np.copy(last_cv_image)

        if cv_image.shape[0]!= constants.camera_pixel_height or cv_image.shape[1]!=constants.camera_pixel_width:
            row_lower = int(cv_image.shape[0]/2 - constants.camera_pixel_height / 2)
            row_upper = int(cv_image.shape[0]/2 + constants.camera_pixel_height / 2)
            col_lower = int(cv_image.shape[1]/2 - constants.camera_pixel_width / 2)
            col_upper = int(cv_image.shape[1]/2 + constants.camera_pixel_width / 2)
            cv_image = cv_image[row_lower:row_upper,col_lower:col_upper,...]
            
        # For YOLO
        resized_yolo_image = cv.resize(cv_image,(constants.yolo_width_of_image,constants.yolo_height_of_image))
        yolo_tensor = torch.from_numpy(resized_yolo_image).to(device).unsqueeze(0).permute(0,3,1,2).to(torch.float).mul(1/256)
        network_prediction = yolo_model(yolo_tensor)
        bounding_boxes = utils.get_bounding_boxes_for_prediction(network_prediction)

        bounding_box_time_end = time.time()
        # scores, x, y, w_h, best_class
        pose_data = np.zeros((0,))
        for bounding_box in bounding_boxes:
            min_x = max(0,bounding_box[1] - bounding_box[3] / 2)
            min_y = max(0,bounding_box[2] - bounding_box[4] / 2)
            max_x = min(1,bounding_box[1] + bounding_box[3] / 2)
            max_y = min(1,bounding_box[2] + bounding_box[4] / 2)
            # if (abs(max_y-min_y) / abs(max_x-min_x))<3 and abs(max_x-min_x) <0.8:
            top_left = (int(min_x * constants.camera_pixel_width),int(min_y * constants.camera_pixel_height))
            bottom_right = (int(max_x * constants.camera_pixel_width),int(max_y * constants.camera_pixel_height))
            
            cv.rectangle(cv_image,top_left,bottom_right,(0,255,0),3)
        last_bbox_image = cv_image
        while (current_time-start_time) / (1/FPS) > running_frame_count:
            running_frame_count+=1
            video_writer.write(cv_image)
            # print(running_frame_count)
        # cv.imshow("Bounding Box",last_bbox_image)
        # cv.waitKey(1)
    
if __name__ == '__main__':
    
    signal.signal(signal.SIGINT,sigint_handler)
    rospy.init_node('bbox_plotter')
    rospy.Subscriber(image_topic, Image, image_callback,queue_size=1)
    print("Starting Recorder.")
    rospy.spin()