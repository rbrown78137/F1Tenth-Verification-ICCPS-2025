from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.pyplot as plt
import matplotlib
# matplotlib.use('Agg')
import verification.collision_verification.collision_probability as collision_probability
import verification.collision_verification.collision_probability_modifications as collision_probability_modifications
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

# Algorithm Settings
braking_forces = [1.6,1.2,0.8, 0.4, 0.2, 0]
# breaking_force_labels = ["100% Max Force","50% Max Force","25% Max Force","12.5% Max Force","No Braking Force"]
braking_forces.reverse()
# breaking_force_labels.reverse()

if __name__ == "__main__":
    # First Clip Tim
    constants.K_STEPS = constants.K_STEPS 
    constants.REACHABILITY_DT = constants.REACHABILITY_DT 
    multiprocessing_cores = 16
    pool = FastPool(multiprocessing_cores)
    lines_to_plot = []
    index_of_interest = 1000
    # index_of_interest = 280
    for braking_idx in range(len(braking_forces)):
        lines_to_plot.append([[],[],[]])
    with open('saved_data/new_video/frame_history_'+str(1)+'.pkl', 'rb') as f:
        history = pickle.load(f)
        for i,frame_data in enumerate(history):
            if i<index_of_interest:
                pose_history = frame_data[1][0]
                actuation_history = frame_data[1][1]
                pose_time_history = frame_data[1][2]
                X_0,sigma_0,U_0 = initial_state.initial_state(pose_history,actuation_history,pose_time_history)
                for braking_idx in range(len(braking_forces)):
                    modified_U_0 = copy.deepcopy(U_0)
                    inputs = []
                    # k,reachability_start_idx,reachability_dt,model_sub_time_steps,X_0,sigma_0,U_0,breaking_acceleration_constant,standard_deviations=6
                    inputs.append([5,0,constants.REACHABILITY_DT,constants.MODEL_SUBTIME_STEPS,X_0,sigma_0,modified_U_0,braking_forces[braking_idx],6])
                    probabilities_of_collision  = pool.map(collision_probability_modifications.breaking_example, inputs)
            
                    
                    lines_to_plot[braking_idx][0].append(frame_data[0] / constants.REACHABILITY_DT)
                    lines_to_plot[braking_idx][1].append(braking_forces[braking_idx] / max(braking_forces) * 100)
                    lines_to_plot[braking_idx][2].append(probabilities_of_collision[0])

        fig = plt.figure()
        ax = Axes3D(fig)
        ax.view_init(20, 60+180) # 30 95
        ax.invert_yaxis()
        # ax.yaxis.set_major_locator(matplotlib.ticker.MaxNLocator(integer=True))
        padding = 8
        ax.xaxis.labelpad=padding
        ax.yaxis.labelpad=padding
        ax.zaxis.labelpad=padding

        ax.set_xlabel("Timesteps Elasped",fontdict={'family' : 'normal',
                    'weight' : 'bold',
                    'size'   : 16})
        ax.set_ylabel("Braking Force (% Max)",fontdict={'family' : 'normal',
                    'weight' : 'bold',
                    'size'   : 16})
        ax.set_zlabel("Probability of Collision",fontdict={'family' : 'normal',
                    'weight' : 'bold',
                    'size'   : 16})
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
        for braking_idx in range(len(braking_forces)):
            timesteps = [time-min_time for time in lines_to_plot[braking_idx][0]]
            braking_forces = lines_to_plot[braking_idx][1]
            probs = lines_to_plot[braking_idx][2]
            colormap = cm.get_cmap('tab10')

            ax.plot3D(timesteps, braking_forces, probs, color=colormap(braking_idx/2 / constants.K_STEPS)) 

        plt.show()
        debug_var = 0
