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

font = {'family' : 'normal',
        'weight' : 'bold',
        'size'   : 16}

matplotlib.rc('font', **font)

RECORDING = 3
CLIP_TIME_DIFF = 0
FUTURE_TIME_STEPS = 10

if RECORDING == 3:
    CLIP_TIME_DIFF = 1682490478.3041098 - 1682489291.685368

if __name__ == "__main__":
    ground_truth_data = None
    prediction_data = None

    with open('saved_data/ground_truth_pose.pkl','rb') as f:
        ground_truth_data = pickle.load(f)

    with open('saved_data/new_video/frame_history_3.pkl','rb') as f:
        prediction_data = pickle.load(f)

    ground_truth_x_positions = []
    ground_truth_y_positions = []

    predicted_x_positions = []
    predicted_y_positions = []

    idx_of_interest = 440
    time_of_prediction = prediction_data[idx_of_interest][0] + CLIP_TIME_DIFF

    X_0, sigma_0, U_0 = initial_state.initial_state(prediction_data[idx_of_interest][1][0],prediction_data[idx_of_interest][1][1],prediction_data[idx_of_interest][1][2])
    predicted_x_positions.append(prediction_data[idx_of_interest][1][0][0][0])
    predicted_y_positions.append(prediction_data[idx_of_interest][1][0][0][1])

    gt_time_idx_pairs = [(abs(ground_truth_data[idx][0]-time_of_prediction),idx) for idx in range(len(ground_truth_data))]
    gt_time_idx_pairs.sort(key=lambda x: x[0])

    corresponding_gt_data = ground_truth_data[gt_time_idx_pairs[0][1]]
    x_ground_truth = corresponding_gt_data[1]
    y_ground_truth = corresponding_gt_data[2]
    ground_truth_x_positions.append(x_ground_truth)
    ground_truth_y_positions.append(y_ground_truth)

    for timestep_idx in range(FUTURE_TIME_STEPS):
        probstar = collision_probability.probstar_next_k_time_steps_given_initial_state(1,timestep_idx,constants.REACHABILITY_DT,constants.MODEL_SUBTIME_STEPS,X_0,sigma_0,U_0,6)[0]
        x_prediction = probstar.V[4][0]
        y_prediction = probstar.V[5][0]
        predicted_x_positions.append(x_prediction)
        predicted_y_positions.append(y_prediction)

        gt_time_idx_pairs = [(ground_truth_data[idx][0]-(time_of_prediction+constants.REACHABILITY_DT*(timestep_idx+1)),idx) for idx in range(len(ground_truth_data))]
        gt_time_idx_pairs.sort(key=lambda x: x[0])
        gt_time_idx_pairs = list(filter(lambda a: a[0]>0, gt_time_idx_pairs))
        corresponding_gt_data = ground_truth_data[gt_time_idx_pairs[0][1]]
        x_ground_truth = corresponding_gt_data[1]
        y_ground_truth = corresponding_gt_data[2]
        ground_truth_x_positions.append(x_ground_truth)
        ground_truth_y_positions.append(y_ground_truth)


    fig = plt.figure()
    ax = fig.add_subplot()

    ax.plot(ground_truth_x_positions, ground_truth_y_positions, marker="s", markersize=10, markeredgecolor=(0,0,0), markerfacecolor=(1.0,1.0,1.0)) 
    ax.plot(predicted_x_positions, predicted_y_positions, marker="o", markersize=10, markeredgecolor=(0,0,0), markerfacecolor=(1.0,1.0,1.0)) 

    legend_elements = [
                       mlines.Line2D([0], [0], color=(0,0,0), marker='s', markersize=12, lw=0, label='Ground Truth Path',markeredgecolor=(0,0,0), markerfacecolor=(1.0,1.0,1.0)),
                       mlines.Line2D([0], [0], color=(0,0,0), marker='o', markersize=12, lw=0, label='Model Path',markeredgecolor=(0,0,0), markerfacecolor=(1.0,1.0,1.0))
                       ]

    ax.legend(handles=legend_elements,loc="upper right")

    fig.set_figheight(7)
    fig.set_figwidth(10)

    ax.set_title("Ground Truth Vs Predictd Model Path",fontdict={'family' : 'normal',
        'weight' : 'bold',
        'size'   : 17})
    ax.set_ylabel("Y (meters)",fontdict={'family' : 'normal',
        'weight' : 'bold',
        'size'   : 17})
    ax.set_xlabel("X (meters)",fontdict={'family' : 'normal',
        'weight' : 'bold',
        'size'   : 17})
    ax.set_ylim(0,1.6)
    ax.set_xlim(-0.6,-0.35)
    # fig.canvas.draw()
    plt.plot()
    plt.show(block=True)
