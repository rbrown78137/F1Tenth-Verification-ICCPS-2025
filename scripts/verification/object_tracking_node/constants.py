import torch
import math

camera_topic = "/camera/color/image_raw"
pose_publish_topic = "/pose_data"
bbox_publish_topic = "/bbox_data"

original_width_image = 640 #1280 #640
original_height_image = 480 #720 #360

yolo_width_of_image = 416
yolo_height_of_image = 416

number_of_classes = 2

split_sizes = [13, 26]
anchor_boxes = [
                [(0.3, 0.24), (0.4, 0.4), (0.7, 0.7)],
                [(0.8, 0.8), (0.12, 0.10), (0.15, 0.10)],
]

# anchor_boxes = [
#                 [(0.28, 0.22), (0.38, 0.48), (0.9, 0.78)],
#                 [(0.07, 0.15), (0.15, 0.11), (0.14, 0.29)],
# ]

camera_pixel_width = original_width_image
camera_pixel_height = original_height_image
yolo_network_width = yolo_width_of_image
yolo_network_height = yolo_height_of_image
pose_network_width = 256
pose_network_height = 256

horizontal_focal_length = 607.300537109375
vertical_focal_length = 607.25244140625
vertical_center = 239.65838623046875
horizontal_center = 317.9561462402344

sampling_size = 10
device = "cuda" if torch.cuda.is_available() else "cpu"

camera_horizontal_focal_length = 674.4192802

def ray_angle(x_center):
    ray = math.atan((0.5-x_center)*camera_pixel_width/horizontal_focal_length)
    return ray


def relative_angle(x_center,ground_truth_yaw):
    return wrapAngle(ground_truth_yaw - ray_angle(x_center))

def global_angle(x_center,relative_angle):
    return wrapAngle(relative_angle + ray_angle(x_center))

def wrapAngle(angle):
    while angle < 0:
        angle += 2 * math.pi
    while angle >= 2 * math.pi:
        angle -= 2 * math.pi
    return angle
