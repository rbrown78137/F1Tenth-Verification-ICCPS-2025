import math
import verification.collision_verification.collision_verification_constants as constants
import verification.collision_verification.kinematic_bicycle_antiderivative as kinematic_bicycle_antiderivative
import numpy as np
import time
from torch.autograd.functional import hessian,jacobian
import torch
import copy
from statistics import mean
# SOLVE_USING_ANGLE_DTHETA = True

def initial_state(prefiltered_pose_history,prefiltered_actuation_history, prefiltered_pose_time_history):
    max_history_to_filter = 20
    pose_history = None
    if len(prefiltered_pose_history) < 20:
        pose_history = copy.deepcopy(prefiltered_pose_history)
        pose_time_history = copy.deepcopy(prefiltered_pose_time_history)
    else:
        pose_history = copy.deepcopy(prefiltered_pose_history[0:20])
        pose_time_history = copy.deepcopy(prefiltered_pose_time_history[0:20])
    start_time_initial_state = time.time()
    mu_zeta_omega,sigma_zeta_omega = 0,0
    mu_V_omega, sigma_V_omega = 0,0
    actuation_history = [[item[0], min(constants.MAX_CAR_STEERING_ANGLE,max(-constants.MAX_CAR_STEERING_ANGLE,item[1])) ] for item in prefiltered_actuation_history[:20]]
    if len(pose_history)>1 and len(pose_history[0])>0:
        mu_zeta_omega,sigma_zeta_omega,mu_V_omega = estimate_omega_velocity_steering_2_points_with_angle(pose_history, actuation_history, pose_time_history)
    # if "-1" in pose_history and "-2" in pose_history and not SOLVE_USING_ANGLE_DTHETA:
    #     mu_zeta_omega,sigma_zeta_omega,mu_V_omega = estimate_omega_velocity_steering_3(pose_history, actuation_history, pose_dt_history)
    # print(f"steering: {round(mu_zeta_omega,3)}, V:{round(mu_V_omega,3)}")
    # print(f"Predicted V:{round(actuation_history[0][0],3)}")
    # print(f"Algo V:{round(mu_V_omega,3)}")
    V_pi = actuation_history[0][0]
    zeta_pi = actuation_history[0][1]

    pi_inputs = [[zeta_pi,constants.PI_AVERAGE_STEERING_ANGLE_ERROR]]
    mu_varphi_x_pi,sigma_varphi_x_pi = approximate_function_distribution(pi_varphi_x_calculation,pi_inputs)
    mu_varphi_y_pi,sigma_varphi_y_pi = approximate_function_distribution(pi_varphi_y_calculation,pi_inputs)
    mu_varphi_x_omega,sigma_varphi_x_omega = 0,0
    mu_varphi_y_omega,sigma_varphi_y_omega = 1,0
    if len(pose_history)>0 and len(pose_history[0])>0:
        dist_theta_i = [pose_history[0][2],pose_history[0][6]]
        dist_theta_j = [pose_history[0][3],pose_history[0][7]]

        dist_theta_omega = theta_distiribution(dist_theta_i,dist_theta_j)
        mu_theta_omega = dist_theta_omega[0]
        sigma_theta_omega = dist_theta_omega[1]

        omega_inputs = [[mu_theta_omega,sigma_theta_omega],[mu_zeta_omega,sigma_zeta_omega]]
        mu_varphi_x_omega,sigma_varphi_x_omega = approximate_function_distribution(omega_varphi_x_calculation,omega_inputs)
        mu_varphi_y_omega,sigma_varphi_y_omega = approximate_function_distribution(omega_varphi_y_calculation,omega_inputs)
        
    mu_x_pi = 0
    mu_y_pi = 0
    mu_x_omega = 0
    mu_y_omega = 0

    if len(pose_history)>0 and len(pose_history[0])>0:
        mu_x_omega = pose_history[0][0]
        mu_y_omega = pose_history[0][1]
    
    sigma_x_pi = 0
    sigma_y_pi = 0
    sigma_x_omega = 0
    sigma_y_omega = 0
    if len(pose_history)>0 and len(pose_history[0])>0:
        sigma_x_omega = pose_history[0][4]
        sigma_y_omega = pose_history[0][5]
    
    X_0 = np.array([mu_x_pi,mu_y_pi,mu_varphi_x_pi,mu_varphi_y_pi,mu_x_omega,mu_y_omega,mu_varphi_x_omega,mu_varphi_y_omega])
    sigma_0 = np.array([sigma_x_pi,sigma_y_pi,sigma_varphi_x_pi,sigma_varphi_y_pi,sigma_x_omega,sigma_y_omega,sigma_varphi_x_omega,sigma_varphi_y_omega])
    U_0 = [V_pi,zeta_pi,mu_V_omega,mu_zeta_omega]
    end_time_initial_state = time.time()
    if constants.LOG_TIMING_INFO:
        print(f"Initial State Time: {end_time_initial_state-start_time_initial_state}")
    return X_0,sigma_0,U_0


def estimate_omega_velocity_steering_2_points_with_angle(pose_history, actuation_history, pose_time_history):
    poses_in_current_frame = translate_past_points_to_current_reference_frame(pose_history, actuation_history, pose_time_history)
    num_predictions = 0 # Number of steering and velocity predictions that can be averaged
    max_predictions_to_average = constants.STEERING_AND_VELOCITY_PREDICTIONS_TO_AVERAGE
    #Calculate num predictions
    for i in range(len(poses_in_current_frame)):
        if pose_time_history[0]-pose_time_history[i] > constants.POSE_DT:
            break
        has_valid_previous_time_step = False
        for j in range(i+1,len(poses_in_current_frame)):
            if pose_time_history[0]-pose_time_history[j] > constants.POSE_DT:
                has_valid_previous_time_step = True
                break
        if has_valid_previous_time_step:
            num_predictions += 1
        else:
            break

    # Find averages steering and velocity predictions for STEERING_AND_VELOCITY_PREDICTIONS_TO_AVERAGE previous time steps
    steering_angles = []
    steering_angle_deviations = []
    velocities = []
    for i in range(min(max_predictions_to_average,num_predictions)):
        previous_time_step_start_idx = -1
        for j in range(i+1,len(poses_in_current_frame)):
            if pose_time_history[0]-pose_time_history[j] > constants.POSE_DT:
                previous_time_step_start_idx = j
                break
        num_current_points = min(constants.POSES_TO_AVERAGE_POSITION,previous_time_step_start_idx)
        num_past_points = min(constants.POSES_TO_AVERAGE_POSITION,len(poses_in_current_frame)-previous_time_step_start_idx)
        if previous_time_step_start_idx <= 0 or num_past_points <=0:
            break

        # Average out position and time over last POSES_TO_AVERAGE_POSITION time steps for calculation of [x_2,y_2,theta_i_2,theta_j_2] and [x_1,y_1,theta_i_1,theta_j_1]
        pose_2_array = [[],[],[],[],[],[],[],[]]
        time_2_array = []
        for current_point_idx in range(num_current_points):
            for pose_item_idx in range(len(pose_history[0])):
                pose_2_array[pose_item_idx].append(poses_in_current_frame[current_point_idx][pose_item_idx])
            time_2_array.append(pose_time_history[current_point_idx])
        pose_1_array = [[],[],[],[],[],[],[],[]]
        time_1_array = []
        for current_point_idx in range(previous_time_step_start_idx,previous_time_step_start_idx+num_past_points):
            for pose_item_idx in range(len(pose_history[0])):
                pose_1_array[pose_item_idx].append(poses_in_current_frame[current_point_idx][pose_item_idx])
            time_1_array.append(pose_time_history[current_point_idx])
        pose_dt = abs(mean(time_2_array)- mean(time_1_array))
        dist_x_2 = [mean(pose_2_array[0]),mean(pose_2_array[4])]
        dist_y_2 = [mean(pose_2_array[1]),mean(pose_2_array[5])]
        dist_theta_i_2 = [mean(pose_2_array[2]),mean(pose_2_array[6])]
        dist_theta_j_2 = [mean(pose_2_array[3]),mean(pose_2_array[7])]
        dist_x_1 = [mean(pose_1_array[0]),mean(pose_1_array[4])]
        dist_y_1 = [mean(pose_1_array[1]),mean(pose_1_array[5])]
        dist_theta_i_1 = [mean(pose_1_array[2]),mean(pose_1_array[6])]
        dist_theta_j_1 = [mean(pose_1_array[3]),mean(pose_1_array[7])]
        # Calculate steering angle and velocity using distributions for two poses [x_2,y_2,theta_i_2,theta_j_2] and [x_1,y_1,theta_i_1,theta_j_1]
        mu_zeta,sigma_zeta,mu_V = 0,0,0
        if is_car_moving(dist_x_2[0],dist_y_2[0],dist_theta_i_2[0],dist_theta_j_2[0],dist_x_1[0],dist_y_1[0],constants.MINIMUM_VELOCITY_CLIPPING_VALUE * pose_dt):
            dist_theta_2 = theta_distiribution(dist_theta_i_2,dist_theta_j_2)
            dist_theta_1 = theta_distiribution(dist_theta_i_1,dist_theta_j_1)
            input_distribution = [dist_x_2, dist_y_2, dist_theta_2, dist_x_1, dist_y_1, dist_theta_1]
            mu_zeta,sigma_zeta = approximate_function_distribution(steering_angle_estimate_2_points_with_angle,input_distribution)
            # Derivative undefined at dtheta = 0
            if abs(mu_zeta) == 0:
                input_distribution = [dist_x_2, dist_y_2, dist_x_1, dist_y_1]
                _,sigma_zeta = approximate_function_distribution(steering_angle_estimate_2_points_same_angle,input_distribution)
            mu_V = velocity_estimate_2_points_with_angle(dist_x_2[0],dist_y_2[0],dist_x_1[0],dist_y_1[0],pose_dt)
            
            # Uncertainty functions do not actually calculate sign of velocity and steering angles so we correct this to make sure the turning direction and velocity are correct
            velocity_sign, steering_sign = velocity_steering_sign(dist_x_1[0],dist_y_1[0],dist_x_2[0],dist_y_2[0],dist_theta_i_1[0],dist_theta_j_1[0], dist_theta_i_2[0],dist_theta_j_2[0])
            mu_zeta = mu_zeta * steering_sign
            mu_V = mu_V * velocity_sign

        steering_angles.append(mu_zeta)
        steering_angle_deviations.append(sigma_zeta)
        velocities.append(mu_V)

    average_steering_angle = 0
    average_steering_angle_deviation = 0
    average_velocity = 0
    if len(steering_angles)>0:
        average_steering_angle = mean(steering_angles) 
        average_steering_angle_deviation = mean(steering_angle_deviations) 
        average_velocity = mean(velocities) 
    # print(*velocities)

    if np.isnan(average_steering_angle):
        mu_zeta,sigma_zeta = non_dist_steering_angle_estimate_2_points_with_angle(*input_distribution)
        debug_var_1 = 0
    if np.isnan(average_steering_angle_deviation):
        debug_var_1 = 0
    return average_steering_angle,average_steering_angle_deviation,average_velocity


def estimate_omega_velocity_steering_3(pose_history, actuation_history, pose_time_history):
    # Fill out with commented equations
    return 0,0,0

def translate_past_points_to_current_reference_frame(pose_history,actuation_history,pose_time_history):
    new_pose_history = copy.deepcopy(pose_history)
    for i in range(len(new_pose_history)-1,-1,-1):
        for j in range(i+1,len(new_pose_history),1):
            # Translate reference frame forward one time step
            L = constants.WHEELBASE
            s_pi = actuation_history[i][1]
            B_pi = math.atan(math.tan(s_pi) / 2)
            V_pi = (actuation_history[i][0] + actuation_history[i][0])/2
            # print(f"Act Data{actuation_history[i][0]}")
            # V_pi = 0
            dt = abs(pose_time_history[i]-pose_time_history[i+1])
        
            # x_y_z = variable x in time step y in reference frame of t=z
            pi_theta_0_0 = math.pi / 2
            pi_x_1_0 = 0
            pi_y_1_0 = V_pi * dt
            pi_theta_1_0 = pi_theta_0_0
            if abs(B_pi) > 1e-4:
                pi_x_1_0 =kinematic_bicycle_antiderivative.antiderivative_x(pi_theta_0_0,V_pi,s_pi,L,dt)
                pi_y_1_0 =kinematic_bicycle_antiderivative.antiderivative_y(pi_theta_0_0,V_pi,s_pi,L,dt) 
                pi_theta_1_0 = pi_theta_0_0 + dt * V_pi * math.tan(s_pi) * math.cos(B_pi) / L
            dtheta = pi_theta_1_0 - pi_theta_0_0

            mu_x, mu_y, mu_theta_i, mu_theta_j, sigma_x, sigma_y, sigma_theta_i, sigma_theta_j = new_pose_history[j]
            new_mu_x = (mu_x-pi_x_1_0) * math.cos(-dtheta) - (mu_y-pi_y_1_0) * math.sin(-dtheta)
            new_mu_y = (mu_x-pi_x_1_0) * math.sin(-dtheta) + (mu_y-pi_y_1_0) * math.cos(-dtheta)
            new_sigma_x = sigma_x * math.cos(-dtheta) - sigma_y * math.sin(-dtheta)
            new_sigma_y = sigma_x * math.sin(-dtheta) + sigma_y * math.cos(-dtheta)

            new_mu_theta_i = mu_theta_i * math.cos(-dtheta) - mu_theta_j * math.sin(-dtheta)
            new_mu_theta_j = mu_theta_i * math.sin(-dtheta) + mu_theta_j * math.cos(-dtheta)
            new_sigma_theta_i = sigma_theta_i * math.cos(-dtheta) - sigma_theta_j * math.sin(-dtheta)
            new_sigma_theta_j = sigma_theta_i * math.sin(-dtheta) + sigma_theta_j * math.cos(-dtheta)
            new_pose_history[j] = [new_mu_x,new_mu_y,new_mu_theta_i,new_mu_theta_j,new_sigma_x,new_sigma_y,new_sigma_theta_i,new_sigma_theta_j]
    return new_pose_history
'''
Approxmiate distribution of function of random variables using taylor polynomials
NOTE: This implementation uses only first order taylor polynomials due to the computational costs of calculating the hessian matrix on the F1Tenth car, but the logic is included for second order taylor polynomials
'''
def approximate_function_distribution(function_to_approximate,input_distributions):
    uncertainty_matrix = np.zeros((len(input_distributions), len(input_distributions)))
    # hessian_matrix = np.zeros((len(input_distributions), len(input_distributions)))
    
    for i in range(len(input_distributions)):
        uncertainty_matrix[i, i] = input_distributions[i][1]**2
    
    input_means = []
    for input_distribution in input_distributions:
        if math.isnan(input_distribution[0]):
            debug_point = 1
        if math.isnan(input_distribution[1]):
            debug_point = 1
        input_means.append(torch.tensor([float(input_distribution[0])]))

    jacobian_tensor_list = jacobian(function_to_approximate, tuple(input_means))
    jacobian_matrix = np.array([tensor.item() for tensor in jacobian_tensor_list])

    # hessian_tensor = hessian(function_to_approximate, tuple(input_means))
    # for row in range(len(input_distributions)):
    #     for col in range(len(input_distributions)):
    #         hessian_matrix[row,col] = hessian_tensor[row][col]

    # First Order Polynomial
    distribution_mu = function_to_approximate(*input_means).item()
    distribution_variance = np.matmul(np.transpose(jacobian_matrix), np.matmul(uncertainty_matrix, jacobian_matrix))
    # Second Order Taylor Polynomial
    # distribution_mu = function_to_approximate(*function_args) + 0.5 * np.trace(np.matmul(hessian_matrix,uncertainty_matrix))
    # distribution_variance = np.matmul(np.transpose(gradient_matrix),np.matmul(uncertainty_matrix,gradient_matrix)) + 0.25 * np.trace(np.matmul(hessian_matrix,uncertainty_matrix))**2
    distribution_standard_deviation = math.sqrt(distribution_variance)
    return distribution_mu, distribution_standard_deviation

def theta_distiribution(dist_theta_i,dist_theta_j):
    uncertainty_matrix = np.zeros((2, 2))
    uncertainty_matrix[0, 0] = dist_theta_i[1]**2
    uncertainty_matrix[1, 1] = dist_theta_j[1]**2

    theta_i = dist_theta_i[0]
    theta_j = dist_theta_j[0]

    jacobian_matrix = None
    if abs(theta_i) < abs(theta_j):
        # Derivative calculated from arcsin(y/sqrt(x^2+y^2))
        dtheta_dtheta_i = 1
        dtheta_dtheta_j = 0
        if theta_j < 1:
            dtheta_dtheta_i = -1 * (theta_j*theta_i)/(math.pow((theta_j**2 + theta_i**2),1.5)*math.sqrt(1-(theta_j**2 / (theta_j**2 + theta_i**2))))
            dtheta_dtheta_j = (theta_i**2)/(math.pow((theta_j**2 + theta_i**2),1.5)*math.sqrt(1-(theta_j**2 / (theta_j**2 + theta_i**2))))
        jacobian_matrix = np.array([dtheta_dtheta_i, dtheta_dtheta_j])
    else:
        # Derivative calculated from arccos(x/sqrt(x^2+y^2))
        dtheta_dtheta_j = 1
        dtheta_dtheta_i = 0
        if theta_i < 1:
            dtheta_dtheta_j = (theta_j*theta_i)/(math.pow((theta_j**2 + theta_i**2),1.5)*math.sqrt(1-(theta_i**2 / (theta_j**2 + theta_i**2))))
            dtheta_dtheta_i = -1 * (theta_j**2)/(math.pow((theta_j**2 + theta_i**2),1.5)*math.sqrt(1-(theta_i**2 / (theta_j**2 + theta_i**2))))
        jacobian_matrix = np.array([dtheta_dtheta_i, dtheta_dtheta_j])
    # First Order Polynomial
    distribution_mu = math.atan2(dist_theta_j[0],dist_theta_i[0])
    distribution_variance = np.matmul(np.transpose(jacobian_matrix), np.matmul(uncertainty_matrix, jacobian_matrix))
    distribution_standard_deviation = math.sqrt(distribution_variance)
    return [distribution_mu, distribution_standard_deviation]

def steering_angle_estimate_2_points_with_angle(x_2,y_2,theta_2,x_1,y_1,theta_1):
    dtheta = torch.abs(theta_2-theta_1)
    k = torch.clamp(torch.sqrt((2-2*torch.cos(dtheta))/((x_2-x_1)**2+(y_2-y_1)**2)),max=constants.MAX_CAR_CURVATURE)
    beta = torch.asin(torch.clamp(constants.WHEELBASE / 2 * k,min=0,max=1))
    zeta = torch.clamp(torch.atan(2*torch.tan(beta)),min=-1*constants.MAX_CAR_STEERING_ANGLE,max=constants.MAX_CAR_STEERING_ANGLE)
    return zeta

def non_dist_steering_angle_estimate_2_points_with_angle(x_2,y_2,theta_2,x_1,y_1,theta_1):
    dtheta = abs(theta_2-theta_1)
    k = max(constants.MAX_CAR_CURVATURE, math.sqrt((2-2*math.cos(dtheta))/((x_2[0]-x_1[0])**2+(y_2[0]-y_1[0])**2)))
    beta = math.asin(min(1,constants.WHEELBASE / 2 * k))
    zeta = math.atan(2*math.tan(beta))
    return zeta

def steering_angle_estimate_2_points_same_angle(x_2,y_2,x_1,y_1):
    dtheta = 1e-4
    k = torch.sqrt((2-2*math.cos(dtheta))/((x_2-x_1)**2+(y_2-y_1)**2))
    zeta = torch.clamp(torch.atan(2*torch.tan(torch.asin(constants.WHEELBASE / 2 * k))),min=-1*constants.MAX_CAR_STEERING_ANGLE,max=constants.MAX_CAR_STEERING_ANGLE)
    return zeta
    
def velocity_estimate_2_points_with_angle(x_2,y_2,x_1,y_1,dt):
    return math.sqrt(((x_2-x_1)**2+(y_2-y_1)**2))/dt


# def steering_angle_estimate_3_points(x1, y1, x2, y2, x3, y3):
#     k = torch.min(torch.tensor(constants.MAX_CAR_CURVATURE),2 * (x1*y2 + x2*y3 + x3*y1 - x2*y1 - x3*y2 - x1*y3) / (torch.sqrt((x1-x2)**2+(y1-y2)**2) * torch.sqrt((x2-x3)**2+(y2-y3)**2) * torch.sqrt((x3-x1)**2+(y3-y1)**2)))
#     zeta = torch.clamp(torch.atan(2*torch.tan(torch.asin(constants.WHEELBASE / 2 * k))),min=-1*constants.MAX_CAR_STEERING_ANGLE,max=constants.MAX_CAR_STEERING_ANGLE)
#     return zeta


# def velocity_estimate_3_points(x1, y1, x2, y2, x3, y3, dt):
#     k = torch.min(torch.tensor(constants.MAX_CAR_CURVATURE),(torch.abs(2 * (x1*y2 - x1*y3 + x2*y3 - x2*y1 + x3*y1 - x3*y2)) / (torch.sqrt((x1-x2)**2+(y1-y2)**2) * torch.sqrt((x2-x3)**2+(y2-y3)**2) * torch.sqrt((x3-x1)**2+(y3-y1)**2))))
#     V = torch.acos(torch.relu(1-torch.sqrt((y3-y1)**2 + (x3-x1)**2)**2*k**2/2)) / (k * dt)
#     return V

# def velocity_estimate_3_points_small_dt(x_3, y_3, x_2, y_2, x_1, y_1, dt):
#     return (torch.sqrt((x_3-x_2)**2 + (y_3-y_2)**2)+ torch.sqrt((x_2-x_1)**2 + (y_1-y_1)**2) ) / (dt)

# def velocity_estimate_2_points_small_dt(x_2, y_2,theta_i_2,theta_j_2, x_1, y_1, dt):
#     dx = x_2 - x_1 
#     dy = y_2 - y_1
#     # projection of vector [dx,dy]^T into [theta_i,theta_j]^T
#     distance_in_car_moving_direction = dx * theta_i_2 + dy * theta_j_2
#     return distance_in_car_moving_direction / dt


def pi_varphi_x_calculation(zeta):
    return torch.cos(torch.pi/2 + torch.atan(torch.tan(zeta)/2))


def pi_varphi_y_calculation(zeta):
    return torch.sin(torch.pi/2 + torch.atan(torch.tan(zeta)/2))


def omega_varphi_x_calculation(theta,zeta):
    return torch.cos(theta + torch.atan(torch.tan(zeta)/2))


def omega_varphi_y_calculation(theta,zeta):
    return torch.sin(theta + torch.atan(torch.tan(zeta)/2))

# THIS EQUATION SHOULD NOT BE USED TO ESTIMATE UNCERTAINTY. DERIVATIVE BLOWS UP WHEN theta_i IS NEAR 0 WHICH IS THE MAJORITY OF USE CASES. USE AN ARCSIN / ARCOS BASED FUNCTION
def theta_calculation(theta_i,theta_j):
    return torch.atan2(theta_j,theta_i)

def different_coordinate_position(x_1,y_1,x_2,y_2,minimum_distance_between_points):
    if math.sqrt((x_1-x_2)**2 + (y_1-y_2)**2)>minimum_distance_between_points:
        return True
    return False

def is_car_moving(x_2,y_2,theta_i_2,theta_j_2,x_1,y_1,minimum_distance_between_points):
    dx = x_1 - x_2 
    dy = y_1 - y_2
    # projection of vector [dx,dy]^T into [theta_i,theta_j]^T
    distance_in_car_moving_direction = dx * theta_i_2 + dy * theta_j_2
    if abs(distance_in_car_moving_direction) > minimum_distance_between_points:
        return True
    return False


def points_colinear(x1, y1, x2, y2, x3, y3):
    if (x1*y2 - x1*y3 + x2*y3 - x2*y1 + x3*y1 - x3*y2) == 0:
        return True
    else:
        return False

def velocity_steering_sign(x_1, y_1, x_2, y_2,theta_i_1,theta_j_1,theta_i_2,theta_j_2):
    velocity_sign = 1
    steering_sign = -1
    dx = x_2-x_1
    dy = y_2-y_1
    different_directions = (dx*theta_i_1 + dy*theta_j_1) < 0
    counter_clock_wise = (theta_i_1*theta_j_2 - theta_i_2*theta_j_1) > 0
    if counter_clock_wise:
        steering_sign = 1
    if different_directions:
        velocity_sign = -1
        steering_sign = steering_sign * -1
    return velocity_sign,steering_sign

if __name__ == "__main__":
    pose_dt_history = [0,1,2]
    pose_data_0 = [0, -4, 0, 1, 0.01, 0.01, 0.01, 0.01]
    actuation_data_0 = [math.pi,0.198364]
    pose_data_neg_1 = [4, 0, 0, 1, 0.01, 0.01, 0.01, 0.01]
    actuation_data_neg_1 = [math.pi,0.198364]
    pose_data_neg_2 = [0, 4, 0, 1, 0.01, 0.01, 0.01, 0.01]
    actuation_data_neg_2 = [math.pi,0.198364]
    pose_history =[pose_data_0, pose_data_neg_1, pose_data_neg_2]
    actuation_history =[actuation_data_0, actuation_data_neg_1, actuation_data_neg_2]
    X_0,sigma_0,U_0 = initial_state(pose_history,actuation_history,pose_dt_history)
    # while(True):
    #     X_0,sigma_0,U_0 = initial_state(pose_history,actuation_history,pose_dt_history)

    pose_dt_history = [0,1,2]
    pose_data_0 = [0, 1.35, 0, 1, 0.01, 0.01, 0.01, 0.01]
    actuation_data_0 = [0.5,0]
    pose_data_neg_1 = [0, 1.4, 0, 1, 0.01, 0.01, 0.01, 0.01]
    actuation_data_neg_1 = [0.5,0]
    pose_data_neg_2 = [0, 1.5, 0, 1, 0.01, 0.01, 0.01, 0.01]
    actuation_data_neg_2 = [0.5,0]
    pose_history =[pose_data_0, pose_data_neg_1, pose_data_neg_2]
    actuation_history =[actuation_data_0, actuation_data_neg_1, actuation_data_neg_2]       
    X_0,sigma_0,U_0 = initial_state(pose_history,actuation_history,pose_dt_history)
    
    debug_var = 0