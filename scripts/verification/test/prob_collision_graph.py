from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.pyplot as plt
import matplotlib
# matplotlib.use('Agg')
import verification.collision_verification.collision_probability as collision_probability
import matplotlib.pyplot as plt
import numpy as np
import verification.collision_verification.initial_state as initial_state
import verification.collision_verification.collision_verification_constants as constants
import matplotlib.lines as mlines
import cv2 as cv
import pickle
from multiprocessing import Pool
import verification.collision_verification.collision_verification_constants as constants
from verification.collision_verification.fast_pool import FastPool
import time
import copy
import matplotlib.cm as cm

font = {'family' : 'normal',
        'weight' : 'bold',
        'size'   : 16}

matplotlib.rc('font', **font)

COLORS = ["blue","red","green","yellow","purple"]
COLOR_CONSTANT = 1
COLOR_MAP = {"blue":[0,0,255/255],"red":[255/255,0,0],"green":[0,127/255,0],"yellow":[255/255,165/255,0],"purple":[127/255,0,127/255]}

if __name__ == "__main__":
    # First Clip Tim
    constants.K_STEPS = constants.K_STEPS
    constants.REACHABILITY_DT = constants.REACHABILITY_DT 
    multiprocessing_cores = 16
    pool = FastPool(multiprocessing_cores)
    lines_to_plot = []
    # index_of_interest = 220
    # index_of_interest = 280
    index_of_interest = 310#[160,310,350]
    for i in range(constants.K_STEPS):
        lines_to_plot.append([[],[],[]])
    with open('saved_data/new_video/frame_history_'+str(7)+'.pkl', 'rb') as f:
        history = pickle.load(f)
        for i,frame_data in enumerate(history):
            if i<index_of_interest:
                pose_history = frame_data[1][0]
                actuation_history = frame_data[1][1]
                pose_time_history = frame_data[1][2]
                X_0,sigma_0,U_0 = initial_state.initial_state(pose_history,actuation_history,pose_time_history)
                inputs = [[1,k,constants.REACHABILITY_DT,constants.MODEL_SUBTIME_STEPS,X_0,sigma_0,U_0] for k in range(constants.K_STEPS)] 
                probabilities_of_collision  = pool.map(collision_probability.multi_core_future_collision_probabilites, inputs)
                for future_step_idx in range(constants.K_STEPS):
                    lines_to_plot[future_step_idx][0].append(frame_data[0] / constants.REACHABILITY_DT)
                    lines_to_plot[future_step_idx][1].append(future_step_idx+1)
                    lines_to_plot[future_step_idx][2].append(probabilities_of_collision[future_step_idx][0])

        fig = plt.figure()
        ax = Axes3D(fig)
        ax.view_init(20, -120) # 30 95
        # ax.yaxis.set_major_locator(matplotlib.ticker.MaxNLocator(integer=True))
        padding = 8
        ax.xaxis.labelpad=padding
        ax.yaxis.labelpad=padding
        ax.zaxis.labelpad=padding
        ax.set_xlabel("Timesteps Elasped",fontdict={'family' : 'normal',
        'weight' : 'bold',
        'size'   : 16})
        ax.set_ylabel("Timesteps to Collision",fontdict={'family' : 'normal',
        'weight' : 'bold',
        'size'   : 16})
        ax.set_yticks(np.arange(0, 5.5, 1))
        ax.set_zlabel("Probability of Collision",fontdict={'family' : 'normal',
        'weight' : 'bold',
        'size'   : 16})
        ax.set_zticks(np.arange(0, 1.1, 0.2))
        ax.set_zlim(0,1.01)
        # fontdict={'family' : 'normal',
        # 'weight' : 'bold',
        # 'size'   : 17}
        # ax.set_xlim(0,2) # Timestep
        # ax.set_ylim(0,5) # TTC
        # ax.set_zlim(0,1) # Prob Collision
        fig.add_axes(ax)
        fig.set_figheight(6)
        fig.set_figwidth(7)
        min_time = min(lines_to_plot[0][0])
        for index in range(constants.K_STEPS):
            timesteps = [time-min_time for time in lines_to_plot[index][0]]
            ttcs = lines_to_plot[index][1]
            probs = lines_to_plot[index][2]
            colormap = cm.get_cmap('tab10')
            ax.plot3D(timesteps, ttcs, probs, color=COLOR_MAP[COLORS[index]]) 
            # ax.plot3D(timesteps, ttcs, probs, color=colormap(index / constants.K_STEPS)) 

        plt.show()
        debug_var = 0
