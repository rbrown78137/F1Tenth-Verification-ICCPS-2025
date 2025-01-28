# import matplotlib.animation as animation
# from matplotlib.patches import Polygon
# import rospy
# from std_msgs.msg import Float64MultiArray
# import matplotlib.pyplot as plt
# import numpy as np
# import math
# from ackermann_msgs.msg import AckermannDriveStamped
# import time
# import signal
# from threading import Thread
# import pickle
# from sensor_msgs.msg import Image
# import cv2 as cv
# import cv_bridge
# import copy
# import torch
# import rospkg
# import verification.verification_node.calculations as calculations
# import verification.verification_node.verification_constants as constants
# import pypoman
# from matplotlib.patches import Polygon
# from numpy import array, dot, hstack
# from pylab import axis, gca
# from scipy.spatial import ConvexHull
# from nav_msgs.msg import Odometry
# import tf.transformations
# from gazebo_msgs.msg import ModelStates

# prob_data = {}
# start_time = None
# running_frame_count = 0
# FPS = 10
# last_bbox_image = None
# video_writer = cv.VideoWriter('/home/ryan/Paper_Video/probability.avi', 
#                          cv.VideoWriter_fourcc(*'MJPG'),
#                          10, (480,480))

# # Record Time
# # writer = animation.FFMpegWriter(fps=10)
# # nav_drive_topic = "/vesc/low_level/ackermann_cmd_mux/output"
# nav_drive_topic = "/vesc/high_level/ackermann_cmd_mux/input/nav_0"
# pose_topic = "/gazebo/model_states"
# odom_topic = "/vesc/odom"

# last_control_data = None
# last_pose_data = None
# global_start_time = time.time()
# fig = plt.figure(figsize=(6,6))
# ax1 = fig.add_subplot(1,1,1)
# ani = None
# colors = ["blue","red","green","yellow","purple"]
# # colors.reverse()
# def sigint_handler(arg1,arg2):
#     rospy.signal_shutdown(reason="Program Terminated")
#     global video_writer
#     video_writer.release()
#     exit(0)

# def contraint_plotter():
#     global last_pose_data, last_control_data,last_set_of_polygons
#     while last_pose_data is None or last_control_data is None:
#         pass
#     while True:
#         if len(last_pose_data) >0:
#             pose_data = np.array_split(last_pose_data, len(last_pose_data)/8)
#             for pose in pose_data:
#                 prob_stars = calculations.probability_collision_next_n_steps(5,0,constants.dt,pose,last_control_data,constants.velocity_estimate)
#                 start_time = time.time()
#                 for index in range(5):
#                     if not ((index+1) in prob_data):
#                         prob_data[(index+1)] = []
#                     prob_data[index+1].append((start_time-global_start_time,prob_stars[index]))
#                 time.sleep(0.01)
#         else:
            
#             start_time = time.time()
#             for index in range(5):
#                 if not ((index+1) in prob_data):
#                     prob_data[(index+1)] = []
#                 prob_data[index+1].append((start_time-global_start_time,0))
#             time.sleep(0.01)

            

# def control_output_callback(navigation_topic):
#     global last_control_data
#     control_data = [navigation_topic.drive.speed, navigation_topic.drive.steering_angle]
#     last_control_data = control_data
#     # print("Saved Control Data")
#     rospy.sleep(0.001)
#     print(last_control_data)

# def pose_callback(pose):
#     global last_pose_data
#     pi_car_yaw = 0
#     omega_car_yaw = 0
#     pi_car_coordinates = (0,0)
#     omega_car_coordinates = (0,0)
#     for index in range(len(pose.name)):
#         if pose.name[index] == "f1tenth_car":
#             quaternion = (
#             pose.pose[index].orientation.x,
#             pose.pose[index].orientation.y,
#             pose.pose[index].orientation.z,
#             pose.pose[index].orientation.w
#             )
#             euler = tf.transformations.euler_from_quaternion(quaternion)
#             pi_car_yaw = euler[2]
#             pi_car_coordinates = (pose.pose[index].position.x,pose.pose[index].position.y)
#         if pose.name[index] == "red_car":
#             quaternion = (
#             pose.pose[index].orientation.x,
#             pose.pose[index].orientation.y,
#             pose.pose[index].orientation.z,
#             pose.pose[index].orientation.w
#             )
#             euler = tf.transformations.euler_from_quaternion(quaternion)
#             omega_car_yaw = euler[2]
#             omega_car_coordinates = (pose.pose[index].position.x,pose.pose[index].position.y)
#     pi_car_yaw = pi_car_yaw - math.pi/2
#     omega_car_yaw = omega_car_yaw - math.pi/2
#     coord_diff = (omega_car_coordinates[0] - pi_car_coordinates[0], omega_car_coordinates[1] - pi_car_coordinates[1])
#     new_angle = constants.wrapAngle(omega_car_yaw - pi_car_yaw)
#     new_coords = (coord_diff[0]*math.cos(pi_car_yaw)+coord_diff[1]*math.sin(pi_car_yaw),-coord_diff[0]*math.sin(pi_car_yaw) + coord_diff[1]*math.cos(pi_car_yaw))
#     last_pose_data = [new_coords[0],new_coords[1],math.sin(new_angle),math.cos(new_angle),0.02, 0.05, 0.08, 0.08]

# def animate(i):
#     print("Reached Animate Function")
#     global video_writer, FPS, running_frame_count, start_time, prob_data,ax1
#     current_time = time.time()
#     if start_time is None and len(prob_data.keys())>0:
#         start_time = current_time
#     prob_at_time = copy.deepcopy(prob_data)
#     ax1.cla()
#     keys = [key for key in prob_at_time]
#     keys.sort()
#     ax1.set_title("Probability of Collision in Next 5 Time Steps")
#     for key in keys:
#         one_time_step_data =prob_at_time[key]
#         x = [item[0] for item in one_time_step_data]
#         y = [item[1] for item in one_time_step_data]
#         if len(x)>50:
#             x = x[-51:-1]
#             y = y[-51:-1]
#         color_to_display = "black"
#         if int(key)-1 < len(colors):
#             color_to_display = colors[int(key)-1]
#         ax1.plot(x,y,color=color_to_display,label=str(key)+" future time steps")
#     ax1.legend(loc="upper left")
#     img = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8, sep='')
#     img = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))
#     img = cv.resize(img,(480,480))
#     img = cv.cvtColor(img,cv.COLOR_RGB2BGR)
#     if len(prob_data.keys())>0:
#         while (current_time-start_time) / (1/FPS) > running_frame_count:
#             running_frame_count+=1
#             video_writer.write(img)
#             print(running_frame_count)
# # def odom_callback(odom_data):
# #     global last_control_data
# #     speed = odom_data.twist.twist.linear.x
# #     rotation = odom_data.twist.twist.angular.z
# #     if rotation <-0.4:
# #         rotation = -0.4
# #     if rotation > 0.4:
# #         rotation = 0.4
# #     control_data = [speed, rotation]
# #     last_control_data = control_data
# #     # print("Saved Control Data")
# #     rospy.sleep(0.001)
# #     print(last_control_data)


# if __name__ == '__main__':
#     signal.signal(signal.SIGINT,sigint_handler)
#     rospy.init_node('constraint_plotter')
#     rospy.Subscriber(pose_topic, ModelStates, pose_callback,queue_size=100)
#     rospy.Subscriber(nav_drive_topic,AckermannDriveStamped, control_output_callback, queue_size=1)
#     # rospy.Subscriber(odom_topic,Odometry,odom_callback,queue_size=1)
#     constraint_display = Thread(target=contraint_plotter)
#     constraint_display.start()
#     # print("Starting Subscribers.")
    	
#     # 180 = 18 seconds
#     ani = animation.FuncAnimation(fig, animate, interval=100,save_count=4000)
#     # ani.save("/home/ryan/Paper_Video/probability.mp4",writer)
#     plt.show()
#     rospy.spin()