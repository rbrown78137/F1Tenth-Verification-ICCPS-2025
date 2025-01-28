import math
from math import atan2,atan,tan,cos
import verification.collision_verification.collision_verification_constants as constants
import numpy as np

# Input: 2 bounding boxes represented as a list of verticies corresponding to the covnex hull of bounding box 
# Output: List of verticies of convex hull of minkowski difference
# Follows planar case algorithm in https://en.wikipedia.org/wiki/Minkowski_addition
def minkowski_difference_2d_convex_hull(bounding_box_verticies_1,bounding_box_verticies_2):
    # Convert to minkowski addition problem
    bounding_box_verticies_2 = [[-1*vert[0],-1*vert[1]] for vert in bounding_box_verticies_2]

    # Sort by Polar Angle
    bounding_box_verticies_1.sort(key=lambda x:atan2(x[1],x[0]))
    bounding_box_verticies_2.sort(key=lambda x:atan2(x[1],x[0]))

    # Create edges
    # [dx,dy,origninal box from which edge belongs to, index of vertex in original box]
    edges = []
    for vert_idx in range(len(bounding_box_verticies_1)):
        next_vert_idx = vert_idx + 1 if vert_idx + 1 < len(bounding_box_verticies_1) else 0
        dx = bounding_box_verticies_1[next_vert_idx][0] - bounding_box_verticies_1[vert_idx][0]
        dy = bounding_box_verticies_1[next_vert_idx][1]- bounding_box_verticies_1[vert_idx][1]
        edges.append([dx,dy,1,vert_idx])

    for vert_idx in range(len(bounding_box_verticies_2)):
        next_vert_idx = vert_idx + 1 if vert_idx + 1 < len(bounding_box_verticies_2) else 0
        dx = bounding_box_verticies_2[next_vert_idx][0] - bounding_box_verticies_2[vert_idx][0]
        dy = bounding_box_verticies_2[next_vert_idx][1] - bounding_box_verticies_2[vert_idx][1]
        edges.append([dx,dy,2,vert_idx])

    # Sort edges by polar angle
    edges.sort(key=lambda x:atan2(x[1],x[0]))

    # Calculate Starting point
    # Stating point is the the sum of position of starting verticies in the first two edges in the list belonging to each bbox
    first_edge_bbox_1 = next((edge for edge in edges if edge[2] == 1), None)
    first_edge_bbox_2 = next((edge for edge in edges if edge[2] == 2), None)
    starting_x = bounding_box_verticies_1[first_edge_bbox_1[3]][0] + bounding_box_verticies_2[first_edge_bbox_2[3]][0]
    starting_y = bounding_box_verticies_1[first_edge_bbox_1[3]][1] + bounding_box_verticies_2[first_edge_bbox_2[3]][1]

    minkowski_sum_bbox = []
    current_point = (starting_x, starting_y)
    for edge in edges:
        # Add dx and dy to current point
        current_point = (current_point[0]+edge[0], current_point[1]+edge[1])
        minkowski_sum_bbox.append(current_point)
    return minkowski_sum_bbox


def convex_hull_vertex_array_to_linear_constraint(convex_hull_array):
    # CX <= D
    # where X = [x,y]^T
    C = np.zeros((0,2))
    d = np.zeros((0,1))
    for idx in range(len(convex_hull_array)):
        C_i = np.zeros((1,2))
        d_i = np.zeros((1,1))
        x1 = convex_hull_array[idx][0]
        y1 = convex_hull_array[idx][1]
        x2 = 0
        y2 = 0
        if(idx == len(convex_hull_array)-1):
            x2 = convex_hull_array[0][0]
            y2 = convex_hull_array[0][1]
        else:
            x2 = convex_hull_array[idx+1][0]
            y2 = convex_hull_array[idx+1][1]
        C_i[0][0] = -(y2-y1)
        C_i[0][1] = x2-x1
        d_i[0][0] = -(y1 * (x2-x1) - x1 * (y2-y1))
        C = np.vstack([C,C_i])
        d = np.vstack([d,d_i])
    return C,d

# Bounding Boxes in this implementation are defined as rectanges rotated to the estimated heading of the vehicle at that time step
def car_bounding_box_verticies(width, length, angle):
    x1 = (length/2) * math.cos(angle) - (width/2) * math.sin(angle)
    y1 = (width/2) * math.cos(angle) + (length/2) * math.sin(angle)
    x2 = -(length/2) * math.cos(angle) - (width/2) * math.sin(angle)
    y2 = (width/2) * math.cos(angle) - (length/2) * math.sin(angle)
    x3 = (length/2) * math.cos(angle) + (width/2) * math.sin(angle)
    y3 = -(width/2) * math.cos(angle) + (length/2) * math.sin(angle)
    x4 = -(length/2) * math.cos(angle) + (width/2) * math.sin(angle)
    y4 = -(width/2) * math.cos(angle) - (length/2) * math.sin(angle)
    car_bounding_box_verticies = [(x1,y1),(x2,y2),(x3,y3), (x4,y4)]
    return car_bounding_box_verticies


# Estimate Heading of both vehicles based on antiderivative of kinematic bicycle model
def predicted_headings(k,reachability_dt,pose_data,V_pi,zeta_pi,V_omega,zeta_omega):
	B_pi = atan(tan(zeta_pi) / 2)
	B_omega = atan(tan(zeta_omega) / 2)
	theta_omega = atan2(pose_data[3],pose_data[2])
	H_pi = math.pi/2 + (V_pi*tan(zeta_pi)*cos(B_pi))/constants.WHEELBASE * k  * reachability_dt
	H_omega = theta_omega + (V_omega*tan(zeta_omega)*cos(B_omega))/constants.WHEELBASE * k  * reachability_dt
	return H_pi, H_omega

# Main collision predicate described in paper
def two_car_predicate(k,reachability_dt,pose_data,V_pi,zeta_pi,V_omega,zeta_omega):
    angle_pi, angle_omega = predicted_headings(k,reachability_dt,pose_data,V_pi,zeta_pi,V_omega,zeta_omega)
    car_pi = car_bounding_box_verticies(constants.PI_CAR_WIDTH, constants.PI_CAR_LENGTH, angle_pi)
    car_omega = car_bounding_box_verticies(constants.OMEGA_CAR_WIDTH, constants.OMEGA_CAR_LENGTH, angle_omega)
    minkowski_difference = minkowski_difference_2d_convex_hull(car_pi,car_omega)
    C_s,d_s = convex_hull_vertex_array_to_linear_constraint(minkowski_difference)
    C = np.hstack([C_s, np.zeros((C_s.shape[0],2)),-1*C_s,np.zeros((C_s.shape[0],2))])
    d=d_s   
    return C,d

# Predicate Used for Plotting / Testing to determine if car y is within the bounds of car omega
def car_pi_within_bounds_of_car_omega(k,reachability_dt,pose_data,V_pi,zeta_pi,V_omega,zeta_omega):
    angle_pi, angle_omega = predicted_headings(k,reachability_dt,pose_data,V_pi,zeta_pi,V_omega,zeta_omega)
    car_omega = car_bounding_box_verticies(constants.OMEGA_CAR_WIDTH, constants.OMEGA_CAR_LENGTH, angle_omega)
    C_s,d_s = convex_hull_vertex_array_to_linear_constraint(car_omega)
    C = np.hstack([C_s, np.zeros((C_s.shape[0],2)),-1*C_s,np.zeros((C_s.shape[0],2))])
    d=d_s   
    return C,d

 
def predicate_next_k_timesteps(k,reachability_dt,X_0,U_0):
    V_pi,zeta_pi,V_omega,zeta_omega = U_0
    predicates = []
    for reachability_timestep_idx in range(k):
        H,g = two_car_predicate(reachability_timestep_idx,reachability_dt,X_0[4:8],V_pi,zeta_pi,V_omega,zeta_omega)
        predicates.append([H,g])
    return predicates