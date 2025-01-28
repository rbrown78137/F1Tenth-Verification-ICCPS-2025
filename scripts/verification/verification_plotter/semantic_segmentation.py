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
import signal

device = "cuda" if torch.cuda.is_available() else "cpu"
bridge = cv_bridge.CvBridge()

start_time = None
running_frame_count = 0
FPS = 10
last_bbox_image = None
video_writer = cv.VideoWriter('/home/ryan/Paper_Video/semantic_segmentation.avi', 
                         cv.VideoWriter_fourcc(*'MJPG'),
                         10, (640,480))

class SemanticSegmentationNode:
    def __init__(self):
        rospy.init_node('object_tracker_estimation')
        rospack = rospkg.RosPack()
        self.semantic_segmentation_model = torch.jit.load(rospack.get_path('f1tenth_verification') + "/models/object_detection/semantic_segmentation.pt")
        self.semantic_segmentation_model.to(device)
        self.width_of_network = 256
        self.height_of_network = 256
        self.subscriber_topic = "/camera/color/image_raw"
        subscriber = rospy.Subscriber(self.subscriber_topic, Image, self.image_callback)
        rospy.spin()

    def image_callback(self,data):
        global video_writer, FPS, running_frame_count, start_time
        current_time = time.time()
        if start_time is None:
            start_time = current_time
        with torch.no_grad():
            cv_image = bridge.imgmsg_to_cv2(data,desired_encoding='passthrough')
            rgb_image = cv_image
            resized_image = cv.resize(rgb_image,(self.width_of_network,self.height_of_network))
            input_to_network = torch.from_numpy(resized_image).permute((2,0,1)).unsqueeze(0)
            input_to_network = input_to_network.type(torch.FloatTensor).to(device)
            guess = self.semantic_segmentation_model(input_to_network).argmax(1).to("cpu")
            output_image = guess.mul(80).permute((1,2,0)).squeeze().type(torch.ByteTensor).numpy()
            output_image = cv.resize(output_image,(640,480))
            output_image = cv.cvtColor(output_image,cv.COLOR_GRAY2RGB)
            while (current_time-start_time) / (1/FPS) > running_frame_count:
                running_frame_count+=1
                video_writer.write(output_image)
                print(running_frame_count)
            # cv.imshow(mat=output_image,winname="Semantic Segmentation Mask")
            # cv.waitKey(1)
            # image_pub.publish(bridge.cv2_to_imgmsg(output_image, "8UC1"))
def sigint_handler(arg1,arg2):
    global video_writer
    video_writer.release()
    rospy.signal_shutdown(reason="Program Terminated")
    # with open('saved_dictionary.pkl', 'wb') as f:
    #     pickle.dump(prob_data, f)
    # print("Ending Program and Saving Probability data to file.")
    exit(0)
if __name__ == '__main__':
    signal.signal(signal.SIGINT,sigint_handler)
    node = SemanticSegmentationNode()