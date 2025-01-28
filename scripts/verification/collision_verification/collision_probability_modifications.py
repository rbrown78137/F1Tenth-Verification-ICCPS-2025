import numpy as np
from verification.collision_verification.probstar import ProbStar
import verification.collision_verification.model_reachability as model_reachability
import verification.collision_verification.collision_predicate as collision_predicate

# Adds breaking to the normal collision probability problem by applying acceleration to V between each model time step
def breaking_example_probstars(k,reachability_start_idx,reachability_dt,model_sub_time_steps,X_0,sigma_0,U_0,breaking_acceleration_constant,standard_deviations=6):
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
			V_pi = max(0,V_pi - breaking_acceleration_constant * model_dt)

	# Assemble probstars k = reachability_start_idx to k = k_max to verify
	for reachability_timestep_idx in range(k):
		# State Space iteration
		for model_timestep_idx in range(model_sub_time_steps):
			A = model_reachability.two_car_A(V_pi,zeta_pi,V_omega,zeta_omega,model_dt)
			c = np.matmul(A,c)
			V = np.matmul(A, V)
			V_pi = max(0,V_pi - breaking_acceleration_constant * model_dt)

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

def breaking_example(k,reachability_start_idx,reachability_dt,model_sub_time_steps,X_0,sigma_0,U_0,breaking_acceleration_constant,standard_deviations=6):
	probstars = breaking_example_probstars(k,reachability_start_idx,reachability_dt,model_sub_time_steps,X_0,sigma_0,U_0,breaking_acceleration_constant,standard_deviations)
	probabilities = []
	for probstar in probstars:
		probabilities.append(probstar.estimateProbability())
	return max(probabilities)