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
import verification.collision_verification.collision_predicate as collision_predicate
import time
import copy
import sys
import os
import pypoman 
import math
import matplotlib.cm as cm

font = {'family' : 'normal',
        'weight' : 'bold',
        'size'   : 16}

matplotlib.rc('font', **font)


COLORS = ["blue","red","green","yellow","purple"]
COLOR_CONSTANT = 1
COLOR_MAP = {"blue":[0,0,255/255],"red":[255/255,0,0],"green":[0,127/255,0],"yellow":[255/255,165/255,0],"purple":[127/255,0,127/255]}

if __name__ == "__main__":
    index_of_interest = 160
    constants.K_STEPS = constants.K_STEPS
    fig = plt.figure()
    ax = Axes3D(fig)
    (15,120)
    ax.view_init(17, -125)
    fig.add_axes(ax)
    ax.set_ylim(0,constants.K_STEPS)
    ax.set_xlim(-0.6,0.6)
    ax.set_zlim(-0.6,0.6)
    padding = 8
    ax.xaxis.labelpad=padding
    ax.yaxis.labelpad=padding
    ax.zaxis.labelpad=padding
    ax.set_xlabel("$X_{\omega} - X_{\pi}$ (meters)",fontdict={'family' : 'normal',
        'weight' : 'bold',
        'size'   : 16})
    ax.set_zlabel("$Y_{\omega} - Y_{\pi}$ (meters)",fontdict={'family' : 'normal',
        'weight' : 'bold',
        'size'   : 16})
    ax.set_ylabel("Future Time Steps",fontdict={'family' : 'normal',
        'weight' : 'bold',
        'size'   : 16})
    test = np.arange(-0.6, 0.6, 0.3)
    ax.set_xticks(np.arange(-0.6, 0.61, 0.3))
    ax.set_zticks(np.arange(-0.6, 0.61, 0.3))
    # plt.xticks(np.arange(min(x), max(x)+1, 1.0))
    lines_to_plot = []
    for i in range(constants.K_STEPS):
        lines_to_plot.append([[],[],[]])
    # Was old video 5 frame 150
    with open('saved_data/new_video/frame_history_'+str(7)+'.pkl', 'rb') as f:
        history = pickle.load(f)
        frame_data =  history[index_of_interest] #history[20] # 150 for 
        pose_history = frame_data[1][0]
        actuation_history = frame_data[1][1]
        pose_time_history = frame_data[1][2]
        X_0,sigma_0,U_0 = initial_state.initial_state(pose_history,actuation_history,pose_time_history)
        predicates = collision_predicate.predicate_next_k_timesteps(constants.K_STEPS,constants.REACHABILITY_DT,X_0,U_0)
        for predicate_idx, predicate in enumerate(predicates):
                    C = predicate[0]
                    d = predicate[1]

                    E = np.zeros((2, 8))
                    E[0, 0] = 1.
                    E[1, 1] = 1.
                    f = np.zeros(2)
                    proj = (E, f)
                    ineq = (C, d)
                    # print("Finding Polytope")
                    sys.stdout = open(os.devnull, 'w')
                    pypoman_verts = pypoman.projection.project_polyhedron(proj,ineq)[0]
                    sys.stdout = sys.__stdout__
                    
                    pypoman_verts.sort(key=lambda x:math.atan2(x[1],x[0]))

                    x = []
                    y = []
                    z = []

                    for vert in pypoman_verts:
                         x.append(vert[0])
                         z.append(vert[1])
                         y.append(predicate_idx)
                    verts = [list(zip(x,y,z))]
                    poly = Poly3DCollection(verts)
                    poly.set_alpha(0.6)
                    
                    colormap = cm.get_cmap('tab10')

                    # poly.set_color(colormap(predicate_idx / constants.K_STEPS))
                    poly.set_color(COLOR_MAP[COLORS[predicate_idx]])
                    ax.add_collection3d(poly)

        plt.show()
        debug_var = 0


# if __name__ == "__main__":
#     fig = plt.figure()
#     ax = Axes3D(fig)
#     ax.view_init(20, -45)
#     ax.set_xlim(0,2) # Timestep
#     ax.set_ylim(0,3) # TTC
#     ax.set_zlim(0,1) # Prob Collision
#     fig.add_axes(ax)
#     x = [0,0,0,0]
#     y = [0,0,1,1]
#     z = [0,1,1,0]
#     for index in range(5):
#         x = [0.1*index,0.1*index,0.1*index,0.1*index]
#         verts = [list(zip(x,y,z))]
#         poly = Poly3DCollection(verts)
#         poly.set_alpha(0.1)
#         ax.add_collection3d(poly)
#     plt.show()
