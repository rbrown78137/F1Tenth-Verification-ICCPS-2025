#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2 as cv
import torch
import verification.object_tracking_node.utils as utils
import verification.object_tracking_node.constants as constants
from std_msgs.msg import Float64MultiArray
import cv_bridge
import math
import numpy as np
import geometry_msgs.msg
import tf.transformations
import time
import rospkg
import random
import matplotlib.pyplot as plt
import statistics
import verification.object_tracking_node.image_processing as image_processing
import time

LOG_RUNTIME = False

device = "cuda" if torch.cuda.is_available() else "cpu"
rospack = rospkg.RosPack()

yolo_model = torch.jit.load(rospack.get_path('f1tenth_verification') + "/models/object_detection/yolo.pt")
yolo_model.to(device)

pose_model = torch.jit.load(rospack.get_path('f1tenth_verification') + "/models/object_detection/pose.pt")
pose_model.to(device)


bridge = cv_bridge.CvBridge()
def image_callback(data,args):
    total_start_time = time.time()
    timestamp = data.header.stamp.to_sec()
    pose_publisher,bbox_publisher = args
    with torch.no_grad():
        cv_image = bridge.imgmsg_to_cv2(data,desired_encoding='rgb8')
        if cv_image.shape[0]!= constants.camera_pixel_height or cv_image.shape[1]!=constants.camera_pixel_width:
            row_lower = int(cv_image.shape[0]/2 - constants.camera_pixel_height / 2)
            row_upper = int(cv_image.shape[0]/2 + constants.camera_pixel_height / 2)
            col_lower = int(cv_image.shape[1]/2 - constants.camera_pixel_width / 2)
            col_upper = int(cv_image.shape[1]/2 + constants.camera_pixel_width / 2)
            cv_image = cv_image[row_lower:row_upper,col_lower:col_upper,...]
        
        # RUN IMAGE THROUGH YOLO SEGMENT
        resized_yolo_image = cv.resize(cv_image,(constants.yolo_width_of_image,constants.yolo_height_of_image))
        yolo_tensor = torch.from_numpy(resized_yolo_image).to(device).unsqueeze(0).permute(0,3,1,2).to(torch.float).mul(1/256)
        start_time_yolo = time.time()
        network_prediction = yolo_model(yolo_tensor)
        bounding_boxes = utils.get_bounding_boxes_for_prediction(network_prediction)
        end_time_yolo = time.time()
        
        final_bboxes = []
        pose_data = np.zeros((0,))
        for bounding_box in bounding_boxes:
            min_x = max(0,bounding_box[1] - bounding_box[3] / 2)
            min_y = max(0,bounding_box[2] - bounding_box[4] / 2)
            max_x = min(1,bounding_box[1] + bounding_box[3] / 2)
            max_y = min(1,bounding_box[2] + bounding_box[4] / 2)
            final_bboxes.append(bounding_box)
            # PREPARES IMAGE AS DESCRIBED IN FIGURE 2
            prepared_image = image_processing.prepare_image(cv_image,min_x,max_x,min_y,max_y)
            pose_start_time = time.time()
            # POSE BODY AND HEAD ARE EXPORTED TO A SINGLE MODEL
            predictions, uncertainty = pose_model(prepared_image.unsqueeze(0))
            new_pose_info = torch.cat((predictions,uncertainty),axis=1).squeeze(0).to("cpu")
            pose_end_time = time.time()
            relative_angle = math.atan2(new_pose_info[...,3:4],new_pose_info[...,2:3]) 
            global_angle = constants.global_angle((min_x + max_x) / 2, relative_angle)
            new_pose_info[...,2:3] = math.cos(global_angle) 
            new_pose_info[...,3:4] = math.sin(global_angle)
            new_pose_info[...,6:7] = math.cos(global_angle) * new_pose_info[...,6:7] - math.sin(global_angle) * new_pose_info[...,7:8]
            new_pose_info[...,7:8] = math.sin(global_angle) * new_pose_info[...,6:7] + math.cos(global_angle) * new_pose_info[...,7:8]
            new_pose_info[...,4:5] = abs(new_pose_info[...,4:5])
            new_pose_info[...,5:6] = abs( new_pose_info[...,5:6])
            pose_data = np.concatenate([pose_data,new_pose_info],axis=0)
        if len(final_bboxes) > 0:
            bbox_publish_data = Float64MultiArray()
            bbox_publish_data.data = [timestamp] + np.concatenate(final_bboxes,axis=0).tolist()

            bbox_publisher.publish(bbox_publish_data)
            pose_publish_data = Float64MultiArray()
            pose_publish_data.data = [timestamp] + pose_data.tolist()
            pose_publisher.publish(pose_publish_data)
        if len(final_bboxes) == 0:
            bbox_publisher.publish(Float64MultiArray(data=[]))
            pose_publisher.publish(Float64MultiArray(data=[]))
    total_end_time = time.time()
    if LOG_RUNTIME:
        print("Total Timing: " + str(total_end_time - total_start_time))
        print("BBOX Network: " + str(end_time_yolo - start_time_yolo))
        print("Pose Network: " + str(pose_end_time - pose_start_time))
def setup():
    rospy.init_node('object_tracker')
    pose_publisher = rospy.Publisher(constants.pose_publish_topic,Float64MultiArray,queue_size=100)
    bbox_publisher = rospy.Publisher(constants.bbox_publish_topic,Float64MultiArray,queue_size=100)
    rospy.Subscriber(constants.camera_topic, Image, image_callback,callback_args=(pose_publisher,bbox_publisher))
    rospy.spin()

if __name__ == '__main__':
    setup()
