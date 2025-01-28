#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TransformStamped
from vicon_bridge.msg import Markers
import signal
from scipy.spatial.transform import Rotation as R
import threading
import math 
import numpy as np
import copy

camera_x_offset = 0.0
camera_y_offset = 0.05
camera_z_offset = 0
pi_real_angle_vicon_diff = 0 

vicon_marker_topic = "/vicon/markers"
car_pi_tracker = "/vicon/RedCar/RedCar"
car_omega_tracker = "/vicon/BlackCar/BlackCar"
camera_topic = "/camera/color/image_raw"

sensor_global_variable_lock = threading.Lock()
global_pi_car_pose = []
global_omega_car_pose = []
global_pi_car_pose_time = []
global_omega_car_pose_time = []

POSE_DT  = 0.3
def sigint_handler(arg1,arg2):
    rospy.signal_shutdown(reason="Program Terminated")
    exit(0)

def pi_car_callback(data):
    nanoseconds = data.header.stamp.nsecs
    seconds = data.header.stamp.secs
    rot_w = data.transform.rotation.w
    rot_x = data.transform.rotation.x
    rot_y = data.transform.rotation.y
    rot_z = data.transform.rotation.z
    pos_x = data.transform.translation.x
    pos_y = data.transform.translation.y
    pos_z = data.transform.translation.z

    pi_rotations = R.from_quat([rot_x,rot_y,rot_z,rot_w]).as_euler('xyz', degrees=False)
    pi_yaw = pi_rotations[2]
    stamp_time = seconds + nanoseconds * 1e-9
    debug_var = 0

    sensor_global_variable_lock.acquire()
    global_pi_car_pose.append([pos_x,pos_y,pos_z,pi_yaw])
    global_pi_car_pose_time.append(stamp_time)
    sensor_global_variable_lock.release()

def omega_car_callback(data):
    nanoseconds = data.header.stamp.nsecs
    seconds = data.header.stamp.secs
    rot_w = data.transform.rotation.w
    rot_x = data.transform.rotation.x
    rot_y = data.transform.rotation.y
    rot_z = data.transform.rotation.z
    pos_x = data.transform.translation.x
    pos_y = data.transform.translation.y
    pos_z = data.transform.translation.z

    pi_rotations = R.from_quat([rot_x,rot_y,rot_z,rot_w]).as_euler('xyz', degrees=False)
    pi_yaw = pi_rotations[2]
    stamp_time = seconds + nanoseconds * 1e-9
    # print(f"OMEGA CAR:(X:{pos_x:.3f},Y:{pos_y:.3f})")
    debug_var = 0

    sensor_global_variable_lock.acquire()
    global_omega_car_pose.append([pos_x,pos_y,pos_z,pi_yaw])
    global_omega_car_pose_time.append(stamp_time)
    sensor_global_variable_lock.release()

def vicon_markers_callback(data):
    frame_number = data.frame_number
    nanoseconds = data.header.stamp.nsecs
    seconds = data.header.stamp.secs
    stamp_time = seconds + nanoseconds * 1e-9
    sensor_global_variable_lock.acquire()
    pi_car_pose = copy.deepcopy(global_pi_car_pose)
    pi_car_pose_time = copy.deepcopy(global_pi_car_pose_time)
    omega_car_pose = copy.deepcopy(global_omega_car_pose)
    if len(global_pi_car_pose) > 0 and len(global_pi_car_pose)>0:
        if global_pi_car_pose_time[len(global_pi_car_pose_time)-1] < global_pi_car_pose_time[len(global_pi_car_pose_time)-1]:
            global_pi_car_pose_time.clear()
            global_omega_car_pose_time.clear()
            global_pi_car_pose.clear()
            global_omega_car_pose.clear()
    sensor_global_variable_lock.release()
    last_pi_pose = None
    last_omega_pose = None
    if len(pi_car_pose) > 0 and len(omega_car_pose)>0:
        last_pi_pose = pi_car_pose[len(pi_car_pose)-1]
        last_omega_pose = omega_car_pose[len(omega_car_pose)-1]
    if last_pi_pose is not None and last_omega_pose is not None:
        # GET RELATIVE POSE
        pose = getPoseArray(last_pi_pose,last_omega_pose)
        float_formatter = "{:.4f}".format
        np.set_printoptions(formatter={'float_kind':float_formatter})
        print(f"Pose{np.array([pose])}")
        # GET VELOCITY OF 
        pose_idx = len(pi_car_pose)-1
        while(pose_idx >0 and (pi_car_pose_time[len(pi_car_pose)-1] - pi_car_pose_time[pose_idx]) < POSE_DT):
            pose_idx = pose_idx-1
        x_1 = pi_car_pose[pose_idx][0]
        y_1 = pi_car_pose[pose_idx][1]
        x_2 = pi_car_pose[len(pi_car_pose)-1][0]
        y_2 = pi_car_pose[len(pi_car_pose)-1][1]
        t_1 = pi_car_pose_time[pose_idx]
        t_2 = pi_car_pose_time[len(pi_car_pose)-1]
        if t_1 == t_2:
            debug_var = 0
        V = velocity_estimate(x_1,y_1,x_2,y_2,t_1,t_2)
        print(f"Velocity Estimate {abs(V):.3f} Time:{t_2:.3f}")

def getPoseArray(pi_pose,omega_pose):
    omega_x = omega_pose[0]
    omega_y = omega_pose[1]
    omega_z = omega_pose[2]
    omega_yaw = omega_pose[3]

    pi_x = pi_pose[0]
    pi_y = pi_pose[1]
    pi_z = pi_pose[2]
    pi_yaw = pi_pose[3]

    total_yaw = wrapAngle(omega_yaw - pi_yaw)
    position_vectors = [omega_pose[0] -pi_pose[0], omega_pose[1] -pi_pose[1], omega_pose[2] -pi_pose[2] ]

    r = R.from_euler('z',-pi_yaw+pi_real_angle_vicon_diff)

    local_position_vector = r.apply(position_vectors)
    local_x = local_position_vector[0]
    local_y = local_position_vector[1]
    local_z = local_position_vector[2]
    #print(['%.3f' % n for n in local_position_vector])
    # Final Pose
    pose = [local_x,local_y,total_yaw]
    return pose

def wrapAngle(angle):
    while angle < 0:
        angle += 2 * math.pi
    while angle >= 2 * math.pi:
        angle -= 2 * math.pi
    return angle

def velocity_estimate(x_1,y_1,x_2,y_2,time_1,time_2):
    return math.sqrt((x_2-x_1)**2+(y_2-y_1)**2) / (time_2-time_1)

def setup():
    rospy.init_node('object_tracker')
    rospy.Subscriber(car_pi_tracker,TransformStamped,pi_car_callback,queue_size=10)
    rospy.Subscriber(car_omega_tracker,TransformStamped,omega_car_callback,queue_size=10)
    rospy.Subscriber(vicon_marker_topic,Markers,vicon_markers_callback,queue_size=10)
    signal.signal(signal.SIGINT,sigint_handler)
    rospy.spin()

if __name__ == '__main__':
    setup()
