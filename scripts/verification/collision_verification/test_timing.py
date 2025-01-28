import time
import verification.collision_verification.collision_probability as collision_probability
import verification.collision_verification.initial_state as initial_state
from fast_pool import FastPool
import pickle

if __name__ == "__main__":
    print("Timings in miliseconds\n")
    reachability_dt = 0.1
    pose_dt_history =[0,0.1,0.2]
    model_sub_time_steps = 10
    pose_data_0 = [0, 1.35, 0, 1, 0.01, 0.01, 0.01, 0.01]
    actuation_data_0 = [0.5,0]
    pose_data_neg_1 = [0, 1.4, 0, 1, 0.01, 0.01, 0.01, 0.01]
    actuation_data_neg_1 = [0.5,0]
    pose_data_neg_2 = [0, 1.5, 0, 1, 0.01, 0.01, 0.01, 0.01]
    actuation_data_neg_2 = [0.5,0]
    pose_history =[pose_data_0,pose_data_neg_1,pose_data_neg_2]
    actuation_history =[actuation_data_0,actuation_data_neg_1, actuation_data_neg_2]
    n = 1
    for i in range(10):
        probabilities_1 = collision_probability.single_thread_future_collision_probabilites(n,0,reachability_dt,model_sub_time_steps,pose_history,actuation_history,pose_dt_history)
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

    # Multiple Core Test Results
    fast_pool = FastPool(20)
    # Sleep for process creation
    time.sleep(25)

    test_time = time.time()
    X_0,sigma_0,U_0 = initial_state.initial_state(pose_history,actuation_history,pose_dt_history)
    inputs = [[n,0,reachability_dt,model_sub_time_steps,X_0,sigma_0,U_0]] * 1
    prob  = fast_pool.map(collision_probability.multi_core_future_collision_probabilites, inputs) 
    test_time = time.time()

    print("\nSleeping for 25 seconds for processes to start up.\n")
    n=1
    start_4 = time.time()
    X_0,sigma_0,U_0 = initial_state.initial_state(pose_history,actuation_history,pose_dt_history)
    inputs = [[n,0,reachability_dt,model_sub_time_steps,X_0,sigma_0,U_0]] * 1
    prob  = fast_pool.map(collision_probability.multi_core_future_collision_probabilites, inputs) 
    end_4 = time.time()
    print(f"Multi-core, 1 Star : {1000*(end_4-start_4)}")

    start_5 = time.time()
    X_0,sigma_0,U_0 = initial_state.initial_state(pose_history,actuation_history,pose_dt_history)
    inputs = [[n,0,reachability_dt,model_sub_time_steps,X_0,sigma_0,U_0]] * 10
    prob  = fast_pool.map(collision_probability.multi_core_future_collision_probabilites, inputs) 
    end_5 = time.time()
    print(f"Multi-core, 10 Star : {1000*(end_5-start_5)}")
    
    start_6 = time.time()
    X_0,sigma_0,U_0 = initial_state.initial_state(pose_history,actuation_history,pose_dt_history)
    inputs = [[n,0,reachability_dt,model_sub_time_steps,X_0,sigma_0,U_0]] * 20
    prob  = fast_pool.map(collision_probability.multi_core_future_collision_probabilites, inputs) 
    end_6 = time.time()
    print(f"Multi-core, 20 Star : {1000*(end_6-start_6)}")

    start_7 = time.time()
    X_0,sigma_0,U_0 = initial_state.initial_state(pose_history,actuation_history,pose_dt_history)
    inputs = [[n,0,reachability_dt,model_sub_time_steps,X_0,sigma_0,U_0]] * 100
    prob  = fast_pool.map(collision_probability.multi_core_future_collision_probabilites, inputs) 
    end_7 = time.time()
    print(f"Multi-core, 100 Star : {1000*(end_7-start_7)}")

    time.sleep(4)
    #initial state creation
    with open('saved_data/old_video/frame_history_3.pkl','rb') as f:
        prediction_data = pickle.load(f)
        for idx in range(20):
            idx_of_interest = 180 # Was 160
            start_time = time.time()
            X_0, sigma_0, U_0 = initial_state.initial_state(prediction_data[idx_of_interest][1][0],prediction_data[idx_of_interest][1][1],prediction_data[idx_of_interest][1][2])
            end_time = time.time()
            print(f"Initial State Time:{1000*(end_time-start_time)}")

    fast_pool.shutdown()
    