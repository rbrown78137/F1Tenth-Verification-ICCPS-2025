from multiprocessing import Pool, Process, Pipe, Queue
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
import os
import verification.verification_node.calculations as calculations 
import matplotlib.pyplot as plt
import numpy as np
from StarV.set.probstar import ProbStar
from gazebo_msgs.msg import ModelStates
import tf.transformations
import verification.verification_node.verification_constants as constants
import math
from ackermann_msgs.msg import AckermannDriveStamped
import time
import signal
from threading import Thread
# camera_topic = "/camera/color/image_raw"
pose_topic = "/pose_data"
# model_topic = "/gazebo/model_states"
publish_collision_topic = "/prob_collision"

processes_to_close = []
previous_control = None
previous_pose = None

def sigint_handler(arg1,arg2):
    for process in processes_to_close:
        process.terminate()
    rospy.signal_shutdown(reason="Program Terminated")
    exit(0)
# def sigterm_handler():
#     rospy   
def calculation_processor(server_queue,result_queue):
    while True:
        
        step_in_future,time_start,last_pose_estimate,last_control,velocity_estimate = server_queue.get()

        print("Pulled Star Task!\n")
        for car_idx in range(0,len(last_pose_estimate),8):
            start_star_time = time.time()
            prob_collisions = calculations.probability_collision_next_n_steps(1,0,constants.dt,last_pose_estimate,last_control,0)
            end_star_time = time.time()
            # print("Time to calculate star: "+ str(end_star_time-start_star_time))
            for prob_collision in prob_collisions:
                data_to_send = [step_in_future,time_start,prob_collision]
                print(f"Found Star Uploading! Time:{end_star_time-start_star_time} End time:{end_star_time}\n")
                result_queue.put(data_to_send)

def prob_star_dispatcher(server_queue,pose_data_queue, control_data_queue):
        global previous_control,previous_pose
        while(True):
            last_pose_data = None
            last_control_data = None
            if not pose_data_queue.empty() and control_data_queue.empty() and previous_control is None:
                last_pose_data = pose_data_queue.get()
                previous_pose = last_pose_data
                last_control_data = previous_control
            elif pose_data_queue.empty() and not control_data_queue.empty() and previous_pose is None:
                last_control_data = control_data_queue.get()
                last_pose_data = previous_pose
            elif not pose_data_queue.empty() and not control_data_queue.empty():
                last_pose_data = pose_data_queue.get()
                last_control_data = control_data_queue.get()
            if not last_control_data == None and not last_pose_data == None:
                time_start = time.time()
                if server_queue.qsize() <= 9 * constants.future_star_calculations:
                    for prob_star_idx in range(constants.future_star_calculations):
                        data_to_send = [prob_star_idx,time_start,last_pose_data,last_control_data,constants.velocity_estimate]
                        if(len(last_pose_data)>0):
                            print(f"Sending Stars to be calculated! Sent at: {time_start}\n")
                            server_queue.put(data_to_send) 

# def result_publisher(ros_publisher,result_queue):
#      while True:
#         received_data = result_queue.get()
#         print("Prob Star publisher got result")
#         publish_data = Float64MultiArray()
#         publish_data.data = received_data
#         ros_publisher.publish(publish_data)
#         print("Prob Star published")

def control_output_callback(navigation_topic,args):
    print("Recieved Control\n")
    control_data_queue,ros_publisher, result_queue = args
        
    # Publish Unpublished Stars
    while not(result_queue.empty()):
        received_data = result_queue.get()
        publish_data = Float64MultiArray(data=received_data)
        print("Publishing Star: "+ str(received_data)+"\n")
        ros_publisher.publish(publish_data)

    # Send control data to queue
    control_data = [navigation_topic.drive.speed, navigation_topic.drive.steering_angle]
    if not control_data_queue.full():
            control_data_queue.put(control_data)
    
    else:
        rospy.sleep(0.05)

def pose_callback(pose_data,args):
    print("Recieved Pose\n")
    pose_data_queue,ros_publisher, result_queue = args
    
    # Publish Unpublished Stars
    while not(result_queue.empty()):
        received_data = result_queue.get()
        publish_data = Float64MultiArray(data=received_data)
        print("Publishing Star: "+ str(received_data)+"\n")
        ros_publisher.publish(publish_data)
    
    # Send control data to queue
    # last_pose_data = [0,1,0,0,0.5,0.5,0.1,0.1]
    if not(pose_data_queue.full()):
        pose_data_queue.put(pose_data.data)
    else:
        rospy.sleep(0.05)
def f(conn):
    print("Process Created")
    while(True):
        test = conn.recv()
        print("Got Data")
        conn.send(calculations.probability_collision_next_n_steps(1,0,constants.dt,[1,1,1,1,0,0,0,0],[0,0],0)) 
        print("Sent Data")
if __name__ == '__main__':
    # number_worker_processes = constants.number_worker_processes
    # processes = []
    # prob_star_server_queue = Queue(maxsize=10*constants.future_star_calculations)
    # prob_star_result_queue = Queue(maxsize=100*constants.future_star_calculations)
    # last_control_queue = Queue(maxsize=2*constants.future_star_calculations)
    # last_pose_queue = Queue(maxsize=2*constants.future_star_calculations)

    # for index in range(number_worker_processes):
    #     star_process = Process(target=calculation_processor, args=(prob_star_server_queue,prob_star_result_queue,))
    #     processes.append(star_process)
    #     processes_to_close.append(star_process)
    #     star_process.start()
    # star_process = Process(target=prob_star_dispatcher, args=(prob_star_server_queue,last_pose_queue, last_control_queue,))
    # processes_to_close.append(star_process)
    # star_process.start()

    # signal.signal(signal.SIGINT,sigint_handler)
    # last_control_queue.put([0,0])
    # last_pose_queue.put([1,1,1,1,0,0,0,0])
    # time.sleep(100)
    start = time.time()
    for x in range(1):
        test =calculations.probability_collision_next_n_steps(1,0,constants.dt,[1,1,1,1,0,0,0,0],[0,0],0)
    end = time.time()
    # print(end-start)
    # time.sleep(100)
    # parent_conns = []
    # child_conns = []
    # processes = []
    # for idx in range(20):
    #     parent_conn, child_conn = Pipe()
    #     parent_conns.append(parent_conn)
    #     child_conns.append(child_conn)
    #     p = Process(target=f, args=(child_conn,))
    #     processes.append(p)
    #     p.start()
    # time.sleep(20)
    # new_start = time.time()
    # for idx in range(5):
    #     parent_conns[idx%20].send([0])
    # for idx in range(5):
    #     test =parent_conns[idx%20].recv()
    # new_end = time.time()
    # print(new_end-new_start)
    # for process in processes:
    #     process.join()
        