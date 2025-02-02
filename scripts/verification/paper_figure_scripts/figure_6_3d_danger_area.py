from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib
# matplotlib.use('Agg')
import verification.collision_verification.collision_probability as collision_probability
import matplotlib.pyplot as plt
import numpy as np
import verification.collision_verification.initial_state as initial_state
import verification.collision_verification.collision_verification_constants as constants
import pickle
from multiprocessing import Pool
import verification.collision_verification.collision_verification_constants as constants
import copy
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import tqdm
font = {'family' : 'normal',
        'weight' : 'bold',
        'size'   : 16}

matplotlib.rc('font', **font)


COLOR_NAMES = ["blue","red","green","yellow","purple"]
COLOR_CONSTANT = 1
COLOR_MAP = {"blue":[0,0,255/255],"red":[255/255,0,0],"green":[0,127/255,0],"yellow":[255/255,165/255,0],"purple":[127/255,0,127/255]}
MARKER_SIZE = 10


Y_RANGE_MIN = 0
Y_RANGE_MAX = 3
X_RANGE_MIN = -1
X_RANGE_MAX = 1
X_DENSITY = 100 # ORIGINAL DENSITY USED IN PAPER
Y_DENSITY = 100 # ORIGINAL DENSITY USED IN PAPER
# X_DENSITY = 10 LESS DENSE RECOMMENDATION FOR TESTING
# Y_DENSITY = 10 LESS DENSE RECOMMENDATION FOR TESTING

if __name__ == "__main__":
    print("Drawing of this graph may take several hours using the original density in the paper. Reduce X_DENSITY and Y_DENSITY to produce a less accurate graph in a shorter time")
    video_recordings = [1] # Was [1,7]
    for video_idx in video_recordings:
        with open('saved_data/paper_samples/frame_history_'+str(video_idx)+'.pkl', 'rb') as f:
            # Video 1: 100, 170, 350
            # Video 5: 
            # Video 7: 160, 310, 350

            indicies_to_record = []
            if video_idx == 1:
                indicies_to_record = [100, 170, 350]
            if video_idx == 7:
                indicies_to_record = [160, 310, 350]
            history = pickle.load(f)
            for index_of_interest in indicies_to_record:
                constants.K_STEPS = constants.K_STEPS
                fig = plt.figure()
                ax = Axes3D(fig,computed_zorder=False)
                (15,120)
                ax.view_init(15, -140)
                fig.add_axes(ax)
                ax.set_zlim(Y_RANGE_MIN,Y_RANGE_MAX)
                ax.set_ylim(0,constants.K_STEPS)
                ax.set_xlim(X_RANGE_MIN,X_RANGE_MAX)
                padding = 8
                ax.xaxis.labelpad=padding
                ax.yaxis.labelpad=padding
                ax.zaxis.labelpad=padding
                ax.set_xlabel("X (meters)",fontdict={'family' : 'normal',
                    'weight' : 'bold',
                    'size'   : 16})
                ax.set_zlabel("Y (meters)",fontdict={'family' : 'normal',
                    'weight' : 'bold',
                    'size'   : 16})
                ax.set_ylabel("Timesteps to Collision",fontdict={'family' : 'normal',
                    'weight' : 'bold',
                    'size'   : 16})
                # legend_elements = [
                #                 mlines.Line2D([0], [0], color=(0,0,0), marker='s', markersize=12, lw=0, label='Ego Vehicle',markeredgecolor=(0,0,0), markerfacecolor=(1.0,1.0,1.0)),
                #                 mlines.Line2D([0], [0], color=(0,0,0), marker='o', markersize=12, lw=0, label='Other Vehicle',markeredgecolor=(0,0,0), markerfacecolor=(1.0,1.0,1.0))
                #                 ]
                # ax.legend(handles=legend_elements,loc="upper right")
                lines_to_plot = []
                for i in range(constants.K_STEPS):
                    lines_to_plot.append([[],[],[]])

                frame_data =  history[index_of_interest]
                pose_history = frame_data[1][0]
                actuation_history = frame_data[1][1]
                pose_time_history = frame_data[1][2]
                original_X_0,sigma_0,original_U_0 = initial_state.initial_state(pose_history,actuation_history,pose_time_history)
                X_0 = copy.deepcopy(original_X_0)
                U_0 = copy.deepcopy(original_U_0)
                draw_other_car = True
                if len(pose_history[0])==0:
                    draw_other_car = False
                for future_timestep_idx in range(constants.K_STEPS):

                    # Create a grid of X and Y values

                    x = np.linspace(X_RANGE_MIN, X_RANGE_MAX, X_DENSITY)
                    y = np.linspace(Y_RANGE_MIN, Y_RANGE_MAX, Y_DENSITY)
                    X, Y = np.meshgrid(x, y)
                    density_map = np.zeros((Y_DENSITY,X_DENSITY))
                    U_0[0] = 0
                    U_0[1] = 0
                    if draw_other_car:
                        for row in range(Y_DENSITY):
                            for col in range(X_DENSITY):
                                x_start = X[row][col]
                                x_end = X[row][col] + (X_RANGE_MAX-X_RANGE_MIN) / X_DENSITY
                                y_start = Y[row][col]
                                y_end = Y[row][col] + (Y_RANGE_MAX-Y_RANGE_MIN) / Y_DENSITY

                                X_0[0] = (x_start + x_end) / 2
                                X_0[1] = (y_start + y_end) / 2
                                density_map[row][col] = collision_probability.probstar_next_k_time_steps_given_initial_state(constants.K_STEPS,0,constants.REACHABILITY_DT,constants.MODEL_SUBTIME_STEPS,X_0,sigma_0,U_0,6)[future_timestep_idx].estimateProbability() 

                    time_step_2d_array = np.ones_like(X) * (future_timestep_idx+1)

                    cmap = plt.get_cmap("viridis")
                    colors = cmap(density_map)
                    colors[...,0] = colors[...,0] * 0 + COLOR_MAP[COLOR_NAMES[future_timestep_idx]][0]
                    colors[...,1] = colors[...,1] * 0 + COLOR_MAP[COLOR_NAMES[future_timestep_idx]][1]
                    colors[...,2] = colors[...,2] * 0 + COLOR_MAP[COLOR_NAMES[future_timestep_idx]][2]
                    colors[...,3] = density_map
                    surface = ax.plot_surface(X, time_step_2d_array, Y, facecolors=colors,zorder=-future_timestep_idx)
                probstars = collision_probability.probstar_next_k_time_steps_given_initial_state(constants.K_STEPS,0,constants.REACHABILITY_DT,constants.MODEL_SUBTIME_STEPS,original_X_0,sigma_0,original_U_0,6)
                for timestep_idx, probstar in enumerate(probstars):
                    x_pi = probstar.V[0][0]
                    y = timestep_idx+1
                    z_pi = probstar.V[1][0] 
                    x_omega = probstar.V[4][0]
                    z_omega = probstar.V[5][0]    
                    ax.plot([x_pi,x_pi],[y,y],[0,z_pi],'--',color=(0,0,0))
                    if draw_other_car:
                        ax.plot([x_omega,x_omega],[y,y],[0,z_omega],'--',color=(0,0,0))
                for timestep_idx, probstar in enumerate(probstars):
                    x_pi = probstar.V[0][0]
                    y = timestep_idx+1
                    z_pi = probstar.V[1][0] 
                    x_omega = probstar.V[4][0]
                    z_omega = probstar.V[5][0]    
                    ax.plot([x_pi],[y],[z_pi], marker="s", markersize=MARKER_SIZE, markeredgecolor=(0,0,0), markerfacecolor=(1.0,1.0,1.0))
                    if draw_other_car:
                        ax.plot([x_omega],[y],[z_omega], marker="o", markersize=MARKER_SIZE, markeredgecolor=(0,0,0), markerfacecolor=(1.0,1.0,1.0))
                plt.show()
    debug_var = 0
