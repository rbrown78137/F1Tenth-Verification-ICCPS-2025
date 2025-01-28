import rospy
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
import numpy as np
import verification.verification_node.verification_constants as constants
import math
from ackermann_msgs.msg import AckermannDriveStamped
import time
import signal
from threading import Thread
import pickle

publish_collision_topic = "/prob_collision"
prob_data = {}
# Load File
# with open('saved_dictionary.pkl', 'rb') as f:
#     loaded_dict = pickle.load(f)
def sigint_handler(arg1,arg2):
    rospy.signal_shutdown(reason="Program Terminated")
    with open('saved_dictionary.pkl', 'wb') as f:
        pickle.dump(prob_data, f)
    print("Ending Program and Saving Probability data to file.")
    exit(0)

def prob_star_callback(prob_collision_data):
    steps_in_future, start_time, prob_collision = prob_collision_data.data
    steps_in_future+=1
    if not (steps_in_future in prob_data):
        prob_data[steps_in_future] = []
    prob_data[steps_in_future].append((start_time,prob_collision))
    
if __name__ == '__main__':
    signal.signal(signal.SIGINT,sigint_handler)
    rospy.init_node('verification_plotter')
    # publisher = rospy.Publisher(publish_collision_topic, Float64MultiArray, queue_size=10,)
    rospy.Subscriber(publish_collision_topic, Float64MultiArray, prob_star_callback,queue_size=1)
    print("Starting Spinner.")
    rospy.spin()