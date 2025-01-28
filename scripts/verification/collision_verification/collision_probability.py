import math
import numpy as np
from verification.collision_verification.probstar import ProbStar
import verification.collision_verification.model_reachability as model_reachability
import verification.collision_verification.collision_predicate as collision_predicate
import verification.collision_verification.initial_state as initial_state
import copy


# Main Quantitative Verification Problem in Paper
def probstar_next_k_time_steps(k,reachability_start_idx,reachability_dt,model_sub_time_steps,pose_history,actuation_history,pose_dt_history,standard_deviations=6):
	model_dt = reachability_dt / model_sub_time_steps

	# Define Distribution for Initial State Space Vector
	X_0,sigma_0,U_0 = initial_state.initial_state(pose_history,actuation_history,pose_dt_history)
	V_pi,zeta_pi,V_omega,zeta_omega = U_0

	# Convert Initial State to Probstar 
	c = (np.expand_dims(X_0, axis=0)).transpose()
	V = np.diag(sigma_0)
	n_sigma = np.diag(np.ones(X_0.shape[0]))
	n_mu = np.zeros(X_0.shape[0])
	l = np.ones(X_0.shape[0]) * standard_deviations * -1
	u = np.ones(X_0.shape[0]) * standard_deviations
	probstars = []

	# Iterate Model Until k = reachability_start_idx
	for reachability_timestep_idx in range(reachability_start_idx):
		for model_timestep_idx in range(model_sub_time_steps):
			A = model_reachability.two_car_A(V_pi,zeta_pi,V_omega,zeta_omega,model_dt)
			c = np.matmul(A, c)
			V = np.matmul(A, V)

	# Assemble probstars k = reachability_start_idx to k = k_max to verify
	for reachability_timestep_idx in range(k):
		# State Space iteration
		for model_timestep_idx in range(model_sub_time_steps):
			A = model_reachability.two_car_A(V_pi,zeta_pi,V_omega,zeta_omega,model_dt)
			c = np.matmul(A,c)
			V = np.matmul(A, V)

		# Apply Collision Bounding Predicate
		H,g = collision_predicate.two_car_predicate(reachability_timestep_idx,reachability_dt,pose_history[0],V_pi,zeta_pi,V_omega,zeta_omega)
		C = np.matmul(H,V)
		d = g-np.matmul(H,c)
		d = np.asarray(d).squeeze()
		c_V_Combine =  np.concatenate([c,V], axis=1)
		c_V_Combine = np.asarray(c_V_Combine)
		V = np.asarray(V)
		C = np.asarray(C)
		probstar = ProbStar(c_V_Combine,C,d,n_mu,n_sigma)
		probstars.append(probstar)
	return probstars

# Main Quantitative Verification Problem in Paper (assumes initial state is precomputed)
def probstar_next_k_time_steps_given_initial_state(k,reachability_start_idx,reachability_dt,model_sub_time_steps,X_0,sigma_0,U_0,standard_deviations=6):
	model_dt = reachability_dt / model_sub_time_steps
	V_pi,zeta_pi,V_omega,zeta_omega = U_0

	# Convert Initial State to Probstar 
	c = (np.expand_dims(X_0, axis=0)).transpose()
	V = np.diag(sigma_0)
	n_sigma = np.diag(np.ones(X_0.shape[0]))
	n_mu = np.zeros(X_0.shape[0])
	l = np.ones(X_0.shape[0]) * standard_deviations * -1
	u = np.ones(X_0.shape[0]) * standard_deviations
	probstars = []

	# Iterate Model Until k = reachability_start_idx
	for reachability_timestep_idx in range(reachability_start_idx):
		for model_timestep_idx in range(model_sub_time_steps):
			A = model_reachability.two_car_A(V_pi,zeta_pi,V_omega,zeta_omega,model_dt)
			c = np.matmul(A, c)
			V = np.matmul(A, V)

	# Assemble probstars k = reachability_start_idx to k = k_max to verify
	for reachability_timestep_idx in range(k):
		# State Space iteration
		for model_timestep_idx in range(model_sub_time_steps):
			A = model_reachability.two_car_A(V_pi,zeta_pi,V_omega,zeta_omega,model_dt)
			c = np.matmul(A,c)
			V = np.matmul(A, V)

		# Apply Collision Bounding Predicate
		H,g = collision_predicate.two_car_predicate(reachability_timestep_idx,reachability_dt,X_0[4:8],V_pi,zeta_pi,V_omega,zeta_omega)
		C = np.matmul(H,V)
		d = g-np.matmul(H,c)
		d = np.asarray(d).squeeze()
		c_V_Combine =  np.concatenate([c,V], axis=1)
		c_V_Combine = np.asarray(c_V_Combine)
		V = np.asarray(V)
		C = np.asarray(C)
		probstar = ProbStar(c_V_Combine,C,d,n_mu,n_sigma)
		probstars.append(probstar)
	return probstars


def single_thread_future_collision_probabilites(k,reachability_start_idx,reachability_dt,model_sub_time_steps,pose_history,actuation_history,pose_dt_history,standard_deviations=6):
	probstars = probstar_next_k_time_steps(k,reachability_start_idx,reachability_dt,model_sub_time_steps,pose_history,actuation_history,pose_dt_history,standard_deviations)
	probabilities = []
	for probstar in probstars:
		probabilities.append(probstar.estimateProbability())
	return probabilities

# Multicore example assumes initial state is precomputed to save computation time
def multi_core_future_collision_probabilites(k,reachability_start_idx,reachability_dt,model_sub_time_steps,X_0,sigma_0,U_0,standard_deviations=6):
	probstars = probstar_next_k_time_steps_given_initial_state(k,reachability_start_idx,reachability_dt,model_sub_time_steps,X_0,sigma_0,U_0,standard_deviations)
	probabilities = []
	for probstar in probstars:
		probabilities.append(probstar.estimateProbability())
	return probabilities
 
# 
def car_omega_probstar_next_k_time_steps(X_MIN,X_MAX,Y_MIN,Y_MAX,k,reachability_start_idx,reachability_dt,model_sub_time_steps,X_0,sigma_0,U_0,standard_deviations):
	model_dt = reachability_dt / model_sub_time_steps
	V_pi,zeta_pi,V_omega,zeta_omega = U_0

	# Convert Initial State to Probstar 
	c = (np.expand_dims(X_0, axis=0)).transpose()
	V = np.diag(sigma_0)
	n_sigma = np.diag(np.ones(X_0.shape[0]))
	n_mu = np.zeros(X_0.shape[0])
	l = np.ones(X_0.shape[0]) * standard_deviations * -1
	u = np.ones(X_0.shape[0]) * standard_deviations
	probstars = []

	# Iterate Model Until k = reachability_start_idx
	for reachability_timestep_idx in range(reachability_start_idx):
		for model_timestep_idx in range(model_sub_time_steps):
			A = model_reachability.two_car_A(V_pi,zeta_pi,V_omega,zeta_omega,model_dt)
			c = np.matmul(A, c)
			V = np.matmul(A, V)

	# Assemble probstars k = reachability_start_idx to k = k_max
	for reachability_timestep_idx in range(k):
		for model_timestep_idx in range(model_sub_time_steps):
			A = model_reachability.two_car_A(V_pi,zeta_pi,V_omega,zeta_omega,model_dt)
			c = np.matmul(A,c)
			V = np.matmul(A, V)

		# Apply Collision Bounding Predicate
		H = np.zeros((4,8))
		H[0,4] = -1
		H[1,4] = 1
		H[2,5] = -1
		H[3,5] = 1
		g = np.expand_dims(np.array([-X_MIN,X_MAX,-Y_MIN,Y_MAX]),axis=0).transpose()
		C = np.matmul(H,V)
		d = g-np.matmul(H,c)
		d = np.asarray(d).squeeze()
		c_V_Combine =  np.concatenate([c,V], axis=1)
		c_V_Combine = np.asarray(c_V_Combine)
		V = np.asarray(V)
		C = np.asarray(C)
		probstar = ProbStar(c_V_Combine,C,d,n_mu,n_sigma)
		probstars.append(probstar)
	return probstars 

def car_pi_probstar_next_k_time_steps(X_MIN,X_MAX,Y_MIN,Y_MAX,k,reachability_start_idx,reachability_dt,model_sub_time_steps,X_0,sigma_0,U_0,standard_deviations):
	model_dt = reachability_dt / model_sub_time_steps
	V_pi,zeta_pi,V_omega,zeta_omega = U_0

	# Add slight uncertainty to x_pi and y_pi to improve visual uncertainty
	# REMOVE THIS UNCERTAINTY IF USING FOR A RUNTIME APPLICATION AND NOT FOR DISPLAY
	copy_sigma_0 = copy.deepcopy(sigma_0)
	copy_sigma_0[0] = sigma_0[0] + 0.01
	copy_sigma_0[1] = sigma_0[1] + 0.01

	# Convert Initial State to Probstar 
	c = (np.expand_dims(X_0, axis=0)).transpose()
	V = np.diag(copy_sigma_0)
	n_sigma = np.diag(np.ones(X_0.shape[0]))
	n_mu = np.zeros(X_0.shape[0])
	l = np.ones(X_0.shape[0]) * standard_deviations * -1
	u = np.ones(X_0.shape[0]) * standard_deviations
	probstars = []

	# Iterate Model Until k = reachability_start_idx
	for reachability_timestep_idx in range(reachability_start_idx):
		for model_timestep_idx in range(model_sub_time_steps):
			A = model_reachability.two_car_A(V_pi,zeta_pi,V_omega,zeta_omega,model_dt)
			c = np.matmul(A, c)
			V = np.matmul(A, V)

	# Assemble probstars k = reachability_start_idx to k = k_max
	for reachability_timestep_idx in range(k):
		for model_timestep_idx in range(model_sub_time_steps):
			A = model_reachability.two_car_A(V_pi,zeta_pi,V_omega,zeta_omega,model_dt)
			c = np.matmul(A,c)
			V = np.matmul(A, V)

		# Apply Collision Bounding Predicate
		H = np.zeros((4,8))
		H[0,0] = -1
		H[1,0] = 1
		H[2,1] = -1
		H[3,1] = 1
		g = np.expand_dims(np.array([-X_MIN,X_MAX,-Y_MIN,Y_MAX]),axis=0).transpose()
		C = np.matmul(H,V)
		d = g-np.matmul(H,c)
		d = np.asarray(d).squeeze()
		c_V_Combine =  np.concatenate([c,V], axis=1)
		c_V_Combine = np.asarray(c_V_Combine)
		V = np.asarray(V)
		C = np.asarray(C)
		probstar = ProbStar(c_V_Combine,C,d,n_mu,n_sigma)
		probstars.append(probstar)
	return probstars 

def probstar_next_k_time_steps_given_initial_state_bounded_pi(X_MIN,X_MAX,Y_MIN,Y_MAX,k,reachability_start_idx,reachability_dt,model_sub_time_steps,X_0,sigma_0,U_0,standard_deviations):
	model_dt = reachability_dt / model_sub_time_steps
	V_pi,zeta_pi,V_omega,zeta_omega = U_0

	# Convert Initial State to Probstar 
	c = (np.expand_dims(X_0, axis=0)).transpose()
	V = np.diag(sigma_0)
	n_sigma = np.diag(np.ones(X_0.shape[0]))
	n_mu = np.zeros(X_0.shape[0])
	l = np.ones(X_0.shape[0]) * standard_deviations * -1
	u = np.ones(X_0.shape[0]) * standard_deviations
	probstars = []

	# Iterate Model Until k = reachability_start_idx
	for reachability_timestep_idx in range(reachability_start_idx):
		for model_timestep_idx in range(model_sub_time_steps):
			A = model_reachability.two_car_A(V_pi,zeta_pi,V_omega,zeta_omega,model_dt)
			c = np.matmul(A, c)
			V = np.matmul(A, V)

	# Assemble probstars k = reachability_start_idx to k = k_max
	for reachability_timestep_idx in range(k):
		for model_timestep_idx in range(model_sub_time_steps):
			A = model_reachability.two_car_A(V_pi,zeta_pi,V_omega,zeta_omega,model_dt)
			c = np.matmul(A,c)
			V = np.matmul(A, V)

		# Apply Collision Bounding Predicate
		H,g = collision_predicate.two_car_predicate(reachability_timestep_idx,reachability_dt,X_0[4:8],V_pi,zeta_pi,V_omega,zeta_omega)
		H = np.vstack([H,[0, 0, 0, 0, -1, 0, 0, 0]])
		g = np.vstack([g,[-X_MIN]])
		H = np.vstack([H,[0, 0, 0, 0, 1, 0, 0, 0]])
		g = np.vstack([g,[X_MAX]])
		H = np.vstack([H,[0, 0, 0, 0, 0, -1, 0, 0]])
		g = np.vstack([g,[-Y_MIN]])
		H = np.vstack([H,[0, 0, 0, 0, 0, 1, 0, 0]])
		g = np.vstack([g,[Y_MAX]])
		
		C = np.matmul(H,V)
		d = g-np.matmul(H,c)
		d = np.asarray(d).squeeze()
		c_V_Combine =  np.concatenate([c,V], axis=1)
		c_V_Combine = np.asarray(c_V_Combine)
		V = np.asarray(V)
		C = np.asarray(C)
		probstar = ProbStar(c_V_Combine,C,d,n_mu,n_sigma)
		probstars.append(probstar)
	return probstars


def estimate_probstar_probability(probstar):
    prob = probstar.estimateProbability()
    return prob

if __name__ == "__main__":
	k_max = 30
	reachability_dt = 0.1
	model_sub_time_steps = 10

	pose_dt_history = {"0 to -1":0.1,"-1 to -2":0.2}
	pose_data_0 = [0, 1.35, 0, 1, 0.01, 0.01, 0.01, 0.01]
	actuation_data_0 = [0.5,0]
	pose_data_neg_1 = [0, 1.4, 0, 1, 0.01, 0.01, 0.01, 0.01]
	actuation_data_neg_1 = [0.5,0]
	pose_data_neg_2 = [0, 1.5, 0, 1, 0.01, 0.01, 0.01, 0.01]
	actuation_data_neg_2 = [0.5,0]
	pose_history ={"0":pose_data_0,"-1":pose_data_neg_1,"-2":pose_data_neg_2}
	actuation_history ={"0":actuation_data_0,"-1":actuation_data_neg_1,"-2":actuation_data_neg_2}
	
	probabilities = single_thread_future_collision_probabilites(k_max,0,reachability_dt,model_sub_time_steps,pose_history,actuation_history,pose_dt_history)
	for i, item in enumerate(probabilities):
		print(f"Test 1: t={i}, p={item}")

	pose_dt_history = {"0 to -1":1,"-1 to -2":1}
	pose_data_0 = [0, -4, 0, 1, 0.01, 0.01, 0.01, 0.01]
	actuation_data_0 = [math.pi,0.198364]
	pose_data_neg_1 = [4, 0, 0, 1, 0.01, 0.01, 0.01, 0.01]
	actuation_data_neg_1 = [math.pi,0.198364]
	pose_data_neg_2 = [0, 4, 0, 1, 0.01, 0.01, 0.01, 0.01]
	actuation_data_neg_2 = [math.pi,0.198364]
	pose_history ={"0":pose_data_0, "-1":pose_data_neg_1, "-2":pose_data_neg_2}
	actuation_history ={"0":actuation_data_0, "-1":actuation_data_neg_1, "-2":actuation_data_neg_2}

	probabilities = single_thread_future_collision_probabilites(k_max,0,reachability_dt,model_sub_time_steps,pose_history,actuation_history,pose_dt_history)
	for i, item in enumerate(probabilities):
		print(f"Test 2: t={i}, p={item}")

	pose_dt_history = {"0 to -1":1,"-1 to -2":1}
	pose_data_0 = [0, 1, 0, 1, 0.01, 0.01, 0.01, 0.01]
	actuation_data_0 = [0.5,0]
	pose_data_neg_1 = [0, 1, 0, 1, 0.01, 0.01, 0.01, 0.01]
	actuation_data_neg_1 = [0.5,0]
	pose_data_neg_2 = [0, 1, 0, 1, 0.01, 0.01, 0.01, 0.01]
	actuation_data_neg_2 = [0.5,0]
	pose_history ={"0":pose_data_0, "-1":pose_data_neg_1, "-2":pose_data_neg_2}
	actuation_history ={"0":actuation_data_0, "-1":actuation_data_neg_1, "-2":actuation_data_neg_2}

	probabilities = single_thread_future_collision_probabilites(k_max,0,reachability_dt,model_sub_time_steps,pose_history,actuation_history,pose_dt_history)
	for i, item in enumerate(probabilities):
		print(f"Test 3: t={i}, p={item}")




# if __name__ == "__main__":
# 	k = 30
# 	reachability_dt = 0.1
# 	pose_dt = 0.1
# 	model_sub_time_steps = 10
# 	pose_data_1 = [0, 0, 0, 1, 0.01, 0.01, 0.01, 0.01]
# 	actuation_data_1 = [0.5,0.1]
# 	pose_data_0 = [0, 0, 0, 1, 0.01, 0.01, 0.01, 0.01]
# 	actuation_data_0 = [0.5,0.1]
# 	probabilities = single_thread_future_collision_probabilites(k,0,reachability_dt,model_sub_time_steps,pose_data_1,actuation_data_1,pose_data_0,actuation_data_0,pose_dt)
# 	for i, item in enumerate(probabilities):
# 		print(f"Test 1: t={i}, p={item}")

# 	k = 40
# 	reachability_dt = 0.1
# 	pose_dt = 0.1
# 	model_sub_time_steps = 10
# 	pose_data_1 = [0, 1.35, 0, 1, 0.01, 0.01, 0.01, 0.01]
# 	actuation_data_1 = [0.5,0]
# 	pose_data_0 = [0, 1.4, 0, 1, 0.01, 0.01, 0.01, 0.01]
# 	actuation_data_0 = [0.5,0]
# 	probabilities = single_thread_future_collision_probabilites(k,0,reachability_dt,model_sub_time_steps,pose_data_1,actuation_data_1,pose_data_0,actuation_data_0,pose_dt)
# 	for i, item in enumerate(probabilities):
# 		print(f"Test 2: t={i}, p={item}")