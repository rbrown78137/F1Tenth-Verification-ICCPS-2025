import time
import verification.collision_verification.collision_probability as collision_probability
import verification.collision_verification.initial_state as initial_state
import verification.collision_verification.collision_verification_constants as constants
from verification.collision_verification.fast_pool import FastPool
import pickle

if __name__ == "__main__":
    # Match Simulator Shape for testing
    constants.PI_CAR_LENGTH = 0.540363 
    constants.PI_CAR_WIDTH = 0.298078
    constants.OMEGA_CAR_LENGTH = 0.540363
    constants.OMEGA_CAR_WIDTH = 0.298078
    print("Timings in miliseconds (please wait 25 seconds for process startup) \n")
    fast_pool = FastPool(20)
    time.sleep(25)

    reachability_dt = 0.25
    pose_dt_history =[0,1,2]
    model_sub_time_steps = 10
    # TRIAL 1
    pose_data_0 = [0, 4+constants.PI_CAR_LENGTH, 0, 1, 0, 0.3, 0, 0]
    actuation_data_0 = [1,0]
    pose_data_neg_1 = [0, 5+constants.PI_CAR_LENGTH, 0, 1, 0, 0.3, 0, 0]
    actuation_data_neg_1 = [1,0]
    pose_data_neg_2 = [0, 6+constants.PI_CAR_LENGTH, 0, 1, 0, 0.3, 0, 0]
    actuation_data_neg_2 = [1,0]
    pose_history =[pose_data_0,pose_data_neg_1,pose_data_neg_2]
    actuation_history =[actuation_data_0,actuation_data_neg_1, actuation_data_neg_2]

    # Run once before timing to improve timing accuracy
    n = 1
    X_0,sigma_0,U_0 = initial_state.initial_state(pose_history,actuation_history,pose_dt_history)
    inputs = [[1,i,reachability_dt,model_sub_time_steps,X_0,sigma_0,U_0] for i in range(30)] # 20, 
    prob  = fast_pool.map(collision_probability.multi_core_future_collision_probabilites, inputs)

    start = time.time()
    X_0,sigma_0,U_0 = initial_state.initial_state(pose_history,actuation_history,pose_dt_history)
    inputs = [[1,i,reachability_dt,model_sub_time_steps,X_0,sigma_0,U_0] for i in range(16)] 
    prob  = fast_pool.map(collision_probability.multi_core_future_collision_probabilites, inputs)
    end = time.time()
    print(f"Simulator Example 1 Probability of Collision (%):{max(prob)}")
    print(f"Simulator Example 1 Time (ms):{1000*(end-start)}")

    # TRIAL 2
    pose_data_0 = [3*constants.PI_CAR_WIDTH, 2+constants.PI_CAR_LENGTH, 0, 1, constants.PI_CAR_WIDTH, 0, 0, 0]
    actuation_data_0 = [1,0]
    pose_data_neg_1 = [3*constants.PI_CAR_WIDTH, 3+constants.PI_CAR_LENGTH, 0, 1, constants.PI_CAR_WIDTH, 0, 0, 0]
    actuation_data_neg_1 = [1,0]
    pose_data_neg_2 = [3*constants.PI_CAR_WIDTH * 0, 4+constants.PI_CAR_LENGTH, 0, 1, constants.PI_CAR_WIDTH, 0, 0, 0]
    actuation_data_neg_2 = [1,0]
    pose_history =[pose_data_0,pose_data_neg_1,pose_data_neg_2]
    actuation_history =[actuation_data_0,actuation_data_neg_1, actuation_data_neg_2]

    start = time.time()
    X_0,sigma_0,U_0 = initial_state.initial_state(pose_history,actuation_history,pose_dt_history)
    inputs = [[1,i,reachability_dt,model_sub_time_steps,X_0,sigma_0,U_0] for i in range(16)] 
    prob  = fast_pool.map(collision_probability.multi_core_future_collision_probabilites, inputs)
    end = time.time()
    print(f"Simulator Example 2 Probability of Collision (%):{max(prob)}")
    print(f"Simulator Example 2 Time (ms):{1000*(end-start)}")

    