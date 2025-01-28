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

cap = cv.VideoCapture('/home/ryan/1bbox.mp4')

fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)

def sigint_handler(arg1,arg2):
    rospy.signal_shutdown(reason="Program Terminated")
    exit(0)  

def animate(i):
    global cap
    if cap.isOpened():
        ret, frame = cap.read()
        cv.imshow(mat=frame,winname='test')
        cv.waitKey(1)

if __name__ == '__main__':
    signal.signal(signal.SIGINT,sigint_handler)
    rospy.init_node('constraint_plotter')
    # writer = animation.FFMpegWriter(fps=10, metadata=dict(artist='Me'), bitrate=1800)
    # # 180 = 18 seconds
    ani = animation.FuncAnimation(fig, animate, interval=33.33)
    # ani.save("/home/ryan/Videos/test.mp4",writer)
    plt.show()
    rospy.spin()