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
from matplotlib import cm
from tqdm import tqdm
COLORS = ["blue","red","green","yellow","purple"]
COLOR_CONSTANT = 1
COLOR_MAP = {"blue":[0,0,255/255],"red":[255/255,0,0],"green":[0,127/255,0],"yellow":[255/255,255/255,0],"purple":[127/255,0,127/255]}

if __name__ == "__main__":
    DENSITY = 40
    # Extend to 10
    constants.K_STEPS = constants.K_STEPS 
    # Balance density
    constants.K_STEPS = constants.K_STEPS * DENSITY
    constants.REACHABILITY_DT = constants.REACHABILITY_DT / DENSITY
    multiprocessing_cores = 16
    pool = FastPool(multiprocessing_cores)
    lines_to_plot = []
    lines_to_plot.append([[],[],[]])
    with open('saved_data/new_video/frame_history_'+str(3)+'.pkl', 'rb') as f:
        history = pickle.load(f)
        for frame_data in tqdm(history):
            pose_history = frame_data[1][0]
            actuation_history = frame_data[1][1]
            pose_time_history = frame_data[1][2]
            if len(pose_history[0])>0:
                X_0,sigma_0,U_0 = initial_state.initial_state(pose_history,actuation_history,pose_time_history)
                inputs = [[1,k,constants.REACHABILITY_DT,constants.MODEL_SUBTIME_STEPS,X_0,sigma_0,U_0] for k in range(constants.K_STEPS)] 
                probabilities_of_collision  = pool.map(collision_probability.multi_core_future_collision_probabilites, inputs)
                for future_step_idx in range(constants.K_STEPS):
                    lines_to_plot[0][0].append((frame_data[0] / constants.REACHABILITY_DT)/DENSITY)
                    lines_to_plot[0][1].append((future_step_idx+1)/DENSITY)
                    lines_to_plot[0][2].append(probabilities_of_collision[future_step_idx][0])
            else:
                for future_step_idx in range(constants.K_STEPS):
                        lines_to_plot[0][0].append((frame_data[0] / constants.REACHABILITY_DT)/DENSITY)
                        lines_to_plot[0][1].append((future_step_idx+1)/DENSITY)
                        lines_to_plot[0][2].append(0)

        fig = plt.figure()
        ax = Axes3D(fig)
        ax.view_init(25, -110) # 30 95
        ax.yaxis.set_major_locator(matplotlib.ticker.MaxNLocator(integer=True))
        ax.set_zlim(0,1)
        ax.set_xlabel("Timesteps Elasped")
        ax.set_ylabel("Timesteps to Collision")
        ax.set_zlabel("Probability of Collision")

        # ax.set_xlim(0,2) # Timestep
        # ax.set_ylim(0,5) # TTC
        # ax.set_zlim(0,1) # Prob Collision
        fig.add_axes(ax)
        min_time = min(lines_to_plot[0][0])

        timesteps = [time-min_time for time in lines_to_plot[0][0]]
        ttcs = lines_to_plot[0][1]
        probs = lines_to_plot[0][2]
        ax.plot_trisurf(timesteps, ttcs, probs,cmap=cm.jet,vmin=0, vmax=1) 

        plt.show()
        debug_var = 0
