import pickle
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.lines as mlines
import cv2 as cv
import pickle
from multiprocessing import Pool
import verification.collision_verification.collision_probability as collision_probability
import verification.collision_verification.initial_state as initial_state
import verification.collision_verification.collision_verification_constants as constants
import math
from statistics import mean

font = {'family' : 'normal',
        'weight' : 'bold',
        'size'   : 16}

matplotlib.rc('font', **font)

RECORDING = 3
CLIP_TIME_DIFF = 0
FUTURE_TIME_STEPS = 10

if RECORDING == 3:
    CLIP_TIME_DIFF = 1682490478.3041098 - 1682489291.685368

def get_veloctiy_estimate(ground_truth_data, index_of_interest):
    t_1 = ground_truth_data[index_of_interest-3][0]
    x_1 = ground_truth_data[index_of_interest-3][1]
    y_1 = ground_truth_data[index_of_interest-3][2]
    t_2 = ground_truth_data[index_of_interest][0]
    x_2 = ground_truth_data[index_of_interest][1]
    y_2 = ground_truth_data[index_of_interest][2]
    return math.sqrt((x_2-x_1)**2 + (y_2-y_1)**2) / (t_2-t_1)
if __name__ == "__main__":
    ground_truth_data = None
    prediction_data = None

    with open('saved_data/ground_truth_pose.pkl','rb') as f:
        ground_truth_data = pickle.load(f)

    with open('saved_data/new_video/frame_history_3.pkl','rb') as f:
        prediction_data = pickle.load(f)

    x_error = []
    y_error = []
    theta_error = []
    varphi_x_error = []
    varphi_y_error = []
    velocity_error = []
    steering_angle_error = []
    gt_steering_angle = []
    for i in range(0,len(prediction_data)-1):
        idx_of_interest = i
        time_of_prediction = prediction_data[idx_of_interest][0] + CLIP_TIME_DIFF

        pose_history = prediction_data[idx_of_interest][1][0]
        X_0, sigma_0, U_0 = initial_state.initial_state(prediction_data[idx_of_interest][1][0],prediction_data[idx_of_interest][1][1],prediction_data[idx_of_interest][1][2])
        if len(pose_history[0])>0:
            predicted_angle = math.atan2(pose_history[0][3],pose_history[0][2])
            predicted_x = X_0[4]
            predicted_y = X_0[5]
            predicted_varphi_x = X_0[6]
            predicted_varphi_y = X_0[7]
            predicted_velocity = U_0[2]
            predicted_steering_angle = U_0[3]

            gt_time_idx_pairs = [(abs(ground_truth_data[idx][0]-time_of_prediction),idx) for idx in range(len(ground_truth_data))]
            gt_time_idx_pairs.sort(key=lambda x: x[0])

            corresponding_gt_data = ground_truth_data[gt_time_idx_pairs[0][1]]
            gt_time_idx_pairs[0][1]

            x_ground_truth = corresponding_gt_data[1]
            y_ground_truth = corresponding_gt_data[2]
            theta_ground_truth = corresponding_gt_data[3] + math.pi/2 - 2*math.pi
            varphi_x_ground_truth = math.cos(theta_ground_truth)
            varphi_y_ground_truth = math.sin(theta_ground_truth)
            steering_angle_ground_truth = 0
            velocity_ground_truth = get_veloctiy_estimate(ground_truth_data,gt_time_idx_pairs[0][1])

            x_error.append(abs(x_ground_truth-predicted_x))
            y_error.append(abs(y_ground_truth-predicted_y))
            theta_error.append(abs(theta_ground_truth-predicted_angle))
            varphi_x_error.append(abs(varphi_x_ground_truth-predicted_varphi_x))
            varphi_y_error.append(abs(varphi_y_ground_truth-predicted_varphi_y))
            steering_angle_error.append(abs(steering_angle_ground_truth-predicted_steering_angle))
            velocity_error.append(abs(velocity_ground_truth-predicted_velocity))

    print(f"X Error: {mean(x_error)}")
    print(f"Y Error: {mean(y_error)}")
    print(f"Theta Error: {mean(theta_error)}")
    print(f"varphi_x Error: {mean(varphi_x_error)}")
    print(f"varphi_y Error: {mean(varphi_y_error)}")
    print(f"Velocity Error: {mean(velocity_error)}")
    print(f"Steering Angle Error: {mean(steering_angle_error)}")