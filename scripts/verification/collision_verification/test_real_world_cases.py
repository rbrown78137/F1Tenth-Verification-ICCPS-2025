import verification.collision_verification.collision_probability as collision_probability
if __name__ == "__main__":
    # k = 5
    # reachability_dt = 0.25
    # model_sub_time_steps = 10
    # pose_history = {'0': [-0.1272968351840973, 1.6006253957748413, 0.2118578404188156, 0.9773005247116089, 0.008690353482961655, 0.038259413093328476, 0.2118578404188156, 0.9773005247116089], '-1': [-0.1294117569923401, 1.606104850769043, 0.20278745889663696, 0.9792227745056152, 0.008762096054852009, 0.0414251834154129, 0.20278745889663696, 0.9792227745056152], '-2': [-0.13323290646076202, 1.6045345067977905, 0.214420884847641, 0.976741373538971, 0.010055504739284515, 0.043648552149534225, 0.214420884847641, 0.976741373538971]}
    # actuation_history = {'0': [0, 0.0], '-1': [0, 0.0], '-2': [0, 0.0]}
    # pose_dt_history = {'0 to -1': 0.28476428985595703, '-1 to -2': 0.254380464553833}
    # probabilities = collision_probability.single_thread_future_collision_probabilites(k,0,reachability_dt,model_sub_time_steps,pose_history,actuation_history,pose_dt_history)
    # for i, item in enumerate(probabilities):
    #     print(f"Test 1: t={i}, p={item}")
    
    k = 5
    reachability_dt = 0.25
    model_sub_time_steps = 10
    pose_history = [[-0.09759541600942612, 1.2427924871444702, 0.2726057469844818, 0.962125837802887, 0.005962889641523361, 0.02295193448662758, 0.016910843551158905, 0.051358744502067566], [-0.11236101388931274, 1.362349271774292, 0.28135165572166443, 0.9596047401428223, 0.01025485247373581, 0.02370724081993103, 0.016208790242671967, 0.05307769030332565], [-0.12117079645395279, 1.5110963582992554, 0.22507919371128082, 0.9743404984474182, 0.0076298899948596954, 0.03362426534295082, 0.013705417513847351, 0.048002928495407104]]
    actuation_history = [[0.9855, -0.0], [0.8239042701721191, -0.0], [0.603754991531372, -0.0]]
    pose_dt_history = [0, 0.2960660457611084, 0.2548179626464844]
    probabilities = collision_probability.single_thread_future_collision_probabilites(k,0,reachability_dt,model_sub_time_steps,pose_history,actuation_history,pose_dt_history)
    for i, item in enumerate(probabilities):
        print(f"Test 2: t={i}, p={item}")
    
    print("\n")

    
