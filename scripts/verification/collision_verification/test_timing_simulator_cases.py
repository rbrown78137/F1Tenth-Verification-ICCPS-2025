import time
import verification.collision_verification.collision_probability as collision_probability
import verification.collision_verification.initial_state as initial_state
import verification.collision_verification.collision_verification_constants as constants
from fast_pool import FastPool
import pickle

if __name__ == "__main__":
    # Match Simulator Shape for testing
    constants.PI_CAR_LENGTH = 0.540363 
    constants.PI_CAR_WIDTH = 0.298078
    constants.OMEGA_CAR_LENGTH = 0.540363
    constants.OMEGA_CAR_WIDTH = 0.298078
    # x .298078 y 0.540363 z:0.259901
    print("Timings in miliseconds (please wait 25 seconds for process startup) \n")
    reachability_dt = 0.25
    # pose_dt_history =[0,1,2]
    # model_sub_time_steps = 10
    # pose_data_0 = [0, 4+0.540363, 0, 1, 0, 0.3, 0, 0]
    # actuation_data_0 = [1,0]
    # pose_data_neg_1 = [0, 5+0.540363, 0, 1, 0, 0.3, 0, 0]
    # actuation_data_neg_1 = [1,0]
    # pose_data_neg_2 = [0, 6+0.540363, 0, 1, 0, 0.3, 0, 0]
    # actuation_data_neg_2 = [1,0]
    pose_dt_history =[0,1,2]
    model_sub_time_steps = 10
    pose_data_0 = [0, 2+0.540363, 0, 1, 0.6, 0, 0, 0]
    actuation_data_0 = [1,0]
    pose_data_neg_1 = [0, 3+0.540363, 0, 1, 0.6, 0, 0, 0]
    actuation_data_neg_1 = [1,0]
    pose_data_neg_2 = [0 * 0, 4+0.540363, 0, 1, 0.6, 0, 0, 0]
    actuation_data_neg_2 = [1,0]
    pose_history =[pose_data_0,pose_data_neg_1,pose_data_neg_2]
    actuation_history =[actuation_data_0,actuation_data_neg_1, actuation_data_neg_2]
    n = 1
    fast_pool = FastPool(20)
    # Sleep for process creation
    time.sleep(2)
    X_0,sigma_0,U_0 = initial_state.initial_state(pose_history,actuation_history,pose_dt_history)
    # for i in range(10):
    #     probabilities_1 = collision_probability.probstar_next_k_time_steps_given_initial_state(k,reachability_start_idx,reachability_dt,model_sub_time_steps,X_0,sigma_0,U_0,standard_deviations=6)
    inputs = [[1,i,reachability_dt,model_sub_time_steps,X_0,sigma_0,U_0] for i in range(30)] # 20, 
    prob  = fast_pool.map(collision_probability.multi_core_future_collision_probabilites, inputs)


    start = time.time()
    X_0,sigma_0,U_0 = initial_state.initial_state(pose_history,actuation_history,pose_dt_history)
    # for i in range(10):
    #     probabilities_1 = collision_probability.probstar_next_k_time_steps_given_initial_state(k,reachability_start_idx,reachability_dt,model_sub_time_steps,X_0,sigma_0,U_0,standard_deviations=6)
    inputs = [[1,i,reachability_dt,model_sub_time_steps,X_0,sigma_0,U_0] for i in range(16)] 
    prob  = fast_pool.map(collision_probability.multi_core_future_collision_probabilites, inputs)
    end = time.time()
    print(f"Simulator Example 1:{prob}")
    print(f"Simulator Example 1 Time:{end-start}")
    # Single Core Test Results
    
    n = 1
    # start_1 = time.time()
    # probabilities_1 = collision_probability.single_thread_future_collision_probabilites(n,0,reachability_dt,model_sub_time_steps,pose_history,actuation_history,pose_dt_history)
    # end_1 = time.time()
    # print(f"One Core, 1 Star : {1000*(end_1-start_1)}")
    # n = 10
    # start_2 = time.time()
    # probabilities_2 = collision_probability.single_thread_future_collision_probabilites(n,0,reachability_dt,model_sub_time_steps,pose_history,actuation_history,pose_dt_history)
    # end_2 = time.time()
    # print(f"One Core, 10 Star : {1000*(end_2-start_2)}")
    # n = 100
    # start_3 = time.time()
    # probabilities_3 = collision_probability.single_thread_future_collision_probabilites(n,0,reachability_dt,model_sub_time_steps,pose_history,actuation_history,pose_dt_history)
    # end_3 = time.time()
    # print(f"One Core, 100 Star : {1000*(end_3-start_3)}")
    # n = 1000
    # start_4 = time.time()
    # probabilities_4 = collision_probability.single_thread_future_collision_probabilites(n,0,reachability_dt,model_sub_time_steps,pose_history,actuation_history,pose_dt_history)
    # end_4 = time.time()
    # print(f"One Core, 1000 Star : {1000*(end_4-start_4)}")

    #initial state creation
    # with open('saved_data/old_video/frame_history_3.pkl','rb') as f:
    #     prediction_data = pickle.load(f)
    #     for idx in range(20):
    #         idx_of_interest = 180 # Was 160
    #         start_time = time.time()
    #         X_0, sigma_0, U_0 = initial_state.initial_state(prediction_data[idx_of_interest][1][0],prediction_data[idx_of_interest][1][1],prediction_data[idx_of_interest][1][2])
    #         end_time = time.time()
    #         print(f"Initial State Time:{1000*(end_time-start_time)}")

    # fast_pool.shutdown()
    