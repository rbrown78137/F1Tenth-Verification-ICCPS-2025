
import numpy as np
import time
from StarV.set.probstar import ProbStar
import math
import verification.verification_node.verification_constants as constants
import verification.verification_node.half_space_linear_constraints as half_space_linear_constraints
def getPlantAMatrix(dt):
	A = np.array([
	[1, 0, dt, 0, dt* dt, 0, 0, 0, 0, 0],
	[0, 1, 0, dt, 0, dt* dt, 0, 0, 0, 0],
	[0, 0, 1, 0, dt, 0, 0, 0, 0, 0],
	[0, 0, 0, 1, 0, dt, 0, 0, 0, 0],
	[0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
	[0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
	[0, 0, 0, 0, 0, 0, 1, 0, dt, 0],
	[0, 0, 0, 0, 0, 0, 0, 1, 0, dt],
	[0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
	[0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
	],ndmin=2)
	return A

# def getPlantBMatrix():
# 	A = np.array([
# 	[1, 0, 0, 0, 0* 0, 0, 0, 0, 0, 0],
# 	[0, 1, 0, 0, 0, 0* 0, 0, 0, 0, 0],
# 	[0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
# 	[0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
# 	[0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
# 	[0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
# 	[0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
# 	[0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
# 	[0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
# 	[0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
# 	],ndmin=2)
# 	return A

def getVelocityOmega(pose_x_1,pose_omega_1,control_data_0,dt):
	rotation = np.array([
	[1, 0, dt, 0, dt* dt, 0, 0, 0, 0, 0],
	[0, 1, 0, dt, 0, dt* dt, 0, 0, 0, 0],
	[0, 0, 1, 0, dt, 0, 0, 0, 0, 0],
	[0, 0, 0, 1, 0, dt, 0, 0, 0, 0],
	[0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
	[0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
	[0, 0, 0, 0, 0, 0, 1, 0, dt, 0],
	[0, 0, 0, 0, 0, 0, 0, 1, 0, dt],
	[0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
	[0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
	],ndmin=2)

def getStateVector(pose_network_data,last_control_data,omega_velocity_estimate):
	L = constants.pi_car_length
	# pose_network_data [P_omega, Z_omega, psi_p, psi_z, std. P_omega ,std. Z_omega,...]
	# last control data [v, theta]
	P_pi = 0
	Z_pi = 0
	dP_pi = last_control_data[0] * math.sin(last_control_data[1])
	dZ_pi = last_control_data[0] * math.cos(last_control_data[1])
	acceleration_pi = math.pow(last_control_data[0],2) / math.sqrt(pow(L/2,2) + pow(L,2)*pow(1 / (math.atan(last_control_data[1])+1e-4),2) )
	ddP_pi = acceleration_pi * math.sin(last_control_data[1])
	ddZ_pi = acceleration_pi * math.cos(last_control_data[1])
	P_omega = pose_network_data[0]
	Z_omega = pose_network_data[1]
	dP_omega = omega_velocity_estimate * pose_network_data[2]
	dZ_omega = omega_velocity_estimate * pose_network_data[3]
	return np.array([[P_pi],[Z_pi],[dP_pi],[dZ_pi],[ddP_pi], [ddZ_pi], [P_omega], [Z_omega], [dP_omega], [dZ_omega] ])

def getInitialBasisVectorMatrix(pose_network_data, last_control_data,omega_velocity_estimate):
	E = 0.05
	L = constants.pi_car_length
	# pose_network_data [P_omega, Z_omega, psi_p, psi_z, std. P_omega ,std. Z_omega,...]
	# last control data [v, theta]
	P_pi = 0
	Z_pi = 0
	dP_pi = E * last_control_data[0] * math.sin(last_control_data[1])
	dZ_pi = E * last_control_data[0] * math.cos(last_control_data[1])
	acceleration_pi = math.pow(last_control_data[0],2) / math.sqrt(pow(L/2,2) + pow(L,2)*pow(1 / (math.atan(last_control_data[1])+4),2) )
	ddP_pi = E * acceleration_pi * math.sin(last_control_data[1])
	ddZ_pi = E * acceleration_pi * math.cos(last_control_data[1])
	P_omega = pose_network_data[4]
	Z_omega = pose_network_data[5]
	dP_omega = omega_velocity_estimate * pose_network_data[6]
	dZ_omega = omega_velocity_estimate * pose_network_data[7]
	return np.diag([P_pi,Z_pi,dP_pi,dZ_pi,ddP_pi, ddZ_pi, P_omega, Z_omega, dP_omega, dZ_omega ])

def getPredictedHeadings(pose_network_data,last_control_data,delta_t):
	L = constants.pi_car_length
	H_pi = last_control_data[0] * delta_t / math.sqrt(pow(L/2,2) + pow(L,2)*pow(1 / (math.atan(last_control_data[1])+1e-4),2) )
	H_omega = math.acos(pose_network_data[3] / (math.sqrt(math.pow(pose_network_data[2],2)+math.pow(pose_network_data[3],2)+1e-4)))
	if pose_network_data[4] <0:
		H_omega = 2 * math.pi - H_omega
	return H_pi,H_omega

def probability_collision_next_n_steps(n,start_n,dt,pose_network_data, last_control_data,omega_velocity_estimate,subSteps=1):
	c = getStateVector(pose_network_data,last_control_data,omega_velocity_estimate)
	V = getInitialBasisVectorMatrix(pose_network_data,last_control_data,omega_velocity_estimate)
	N = np.diag([1,1,1,1,1,1,1,1,1,1])
	mu = np.array([0,0,0,0,0,0,0,0,0,0])
	l = np.array([-4,-4,-4,-4,-4,-4,-4,-4,-4,-4])
	u = np.array([4, 4, 4, 4, 4, 4, 4, 4, 4, 4])
	A = getPlantAMatrix(dt)
	probability_collisions = []
	for state_space_iteration in range(start_n):
		c = np.matmul(np.linalg.matrix_power(A,subSteps), c)
		V = np.matmul(np.linalg.matrix_power(A,subSteps), V)
	for prob_star_idx in range(n):
		c = np.matmul(np.linalg.matrix_power(A,subSteps), c)
		V = np.matmul(np.linalg.matrix_power(A,subSteps), V)
		c_V_Combine =  np.concatenate([c,V], axis=1)
		H_pi, H_omega = getPredictedHeadings(pose_network_data,last_control_data,dt * (prob_star_idx+1))
		H, g = half_space_linear_constraints.simple_2_car_predicate(H_pi,H_omega)
		C = np.matmul(H,V)
		d = g-np.matmul(H,c)
		d = d.squeeze()
		prob_star = ProbStar(c_V_Combine,C,d,mu,N,l,u)
		probability_collision = prob_star.estimateProbability()
		probability_collisions.append(probability_collision)
	return probability_collisions

def prob_star_next_time_steps(n,dt,pose_network_data, last_control_data,omega_velocity_estimate,subSteps=1):
	c = getStateVector(pose_network_data,last_control_data,omega_velocity_estimate)
	V = getInitialBasisVectorMatrix(pose_network_data,last_control_data,omega_velocity_estimate)
	N = np.diag([1,1,1,1,1,1,1,1,1,1])
	mu = np.array([0,0,0,0,0,0,0,0,0,0])
	l = np.array([-4,-4,-4,-4,-4,-4,-4,-4,-4,-4])
	u = np.array([4, 4, 4, 4, 4, 4, 4, 4, 4, 4])
	A = getPlantAMatrix(dt)
	prob_stars = []
	for prob_star_idx in range(n):
		c = np.matmul(np.linalg.matrix_power(A,subSteps), c)
		V = np.matmul(np.linalg.matrix_power(A,subSteps), V)
		c_V_Combine =  np.concatenate([c,V], axis=1)
		H_pi, H_omega = getPredictedHeadings(pose_network_data,last_control_data,dt * (prob_star_idx+1))
		H, g = half_space_linear_constraints.simple_2_car_predicate(H_pi,H_omega)
		C = np.matmul(H,V)
		d = g-np.matmul(H,c)
		d = d.squeeze()
		prob_star = ProbStar(c_V_Combine,C,d,mu,N,l,u)
		#prob_star = [c, V, C, d, mu, N, l, u]
		prob_stars.append(prob_star)
	return prob_stars