# import rospy
# from std_msgs.msg import Float64MultiArray
# import matplotlib.pyplot as plt
# import numpy as np
# import verification.verification_node.verification_constants as constants
# import math
# from ackermann_msgs.msg import AckermannDriveStamped
# import time
# import signal
# from threading import Thread
# import pickle
# import matplotlib.animation as animation
# import copy

# publish_collision_topic = "/prob_collision"
# prob_data = {}

# prob_data_display_length = 100
# fig = plt.figure()
# ax1 = fig.add_subplot(1,1,1)
# colors = ["blue","red","green","yellow","purple"]
# global_start_time = None
# # Load File
# # with open('saved_dictionary.pkl', 'rb') as f:
# #     loaded_dict = pickle.load(f)
# def sigint_handler(arg1,arg2):
#     rospy.signal_shutdown(reason="Program Terminated")
#     with open('saved_dictionary.pkl', 'wb') as f:
#         pickle.dump(prob_data, f)
#     print("Ending Program and Saving Probability data to file.")
#     exit(0)

# def prob_star_callback(prob_collision_data):
#     global prob_data, global_start_time
#     print("Got Data")
#     steps_in_future, start_time, prob_collision = prob_collision_data.data
#     if global_start_time is None:
#         global_start_time = start_time
#     steps_in_future+=1
#     if not (steps_in_future in prob_data):
#         prob_data[steps_in_future] = []
#     prob_data[steps_in_future].append((start_time-global_start_time,prob_collision))
# def animate(i):
#     global prob_data,ax1
#     print("Got to animation")
#     prob_at_time = copy.deepcopy(prob_data)
#     ax1.cla()
#     keys = [key for key in prob_at_time]
#     keys.sort()
#     for key in keys:
#         one_time_step_data =prob_at_time[key]
#         x = [item[0] for item in one_time_step_data]
#         y = [item[1] for item in one_time_step_data]
#         if len(x)>prob_data_display_length:
#             x = x[-prob_data_display_length -1:-1]
#             y = y[-prob_data_display_length -1:-1]
#         color_to_display = "black"
#         if int(key)-1 < len(colors):
#             color_to_display = colors[int(key)-1]
#         ax1.plot(x,y,color=color_to_display,label=str(key)+" future time steps")
#     ax1.legend(loc="upper left")
#     # ax1.set_title("Probability of Collision in Future Time Steps")
#     # ax1.set_ylabel("Time step")
#     # ax1.set_xlabel("Probability Collision")
#     # ax1.legend(loc="upper left")


# # def live_prob_plotter():
# #     global prob_data,fig
# #     ani = animation.FuncAnimation(fig, animate, interval=1000)

# if __name__ == '__main__':
#     signal.signal(signal.SIGINT,sigint_handler)
#     rospy.init_node('verification_plotter')
#     rospy.Subscriber(publish_collision_topic, Float64MultiArray, prob_star_callback,queue_size=1)
#     # live_display_thread = Thread(target=live_prob_plotter)
#     # live_display_thread.start()
#     ani = animation.FuncAnimation(fig, animate, interval=100)
#     plt.show()
#     print("Starting Spinner.")
#     rospy.spin()