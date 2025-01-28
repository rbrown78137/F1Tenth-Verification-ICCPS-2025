import matplotlib.pyplot as plt
import pickle
import verification.verification_node.calculations as calculations
# if __name__ =="__main__":
#     with open('saved_dictionary.pkl', 'rb') as f:
#         loaded_dict = pickle.load(f)
#         one_future_time_step =loaded_dict[1]
#         two_future_time_step =loaded_dict[2]
#         three_future_time_step =loaded_dict[3]
#         four_future_time_step =loaded_dict[4]
#         five_future_time_step =loaded_dict[5]
#         x_1 = [item[0] for item in one_future_time_step]
#         y_1 = [item[1] for item in one_future_time_step]
#         x_2 = [item[0] for item in two_future_time_step]
#         y_2 = [item[1] for item in two_future_time_step]
#         x_3 = [item[0] for item in three_future_time_step]
#         y_3 = [item[1] for item in three_future_time_step]
#         x_4 = [item[0] for item in four_future_time_step]
#         y_4 = [item[1] for item in four_future_time_step]
#         x_5 = [item[0] for item in five_future_time_step]
#         y_5 = [item[1] for item in five_future_time_step]
        # plt.plot(x_1,y_1,color="red", label="1 Time Step")
        # plt.plot(x_2,y_2,color="blue", label="2 Time Steps")
        # plt.plot(x_3,y_3,color="green", label="3 Time Steps")
        # # plt.plot(x_4,y_4,color="purple", label="4 Time Steps")
        # # plt.plot(x_5,y_5,color="yellow", label="5 Time Steps")
        # #plt.plot(x,collisions_2,color="blue")
        # #plt.plot(x,collisions_3,color="green")
        # # plt.plot(x,collisions,color="red")
        # plt.xlabel("Time Steps in Future")
        # plt.ylabel("Probability Of Collision")
        # plt.legend(loc = "upper left")
        # plt.show()

if __name__ == '__main__':
    star_calculations = 100
    x = 1
    collisions_1 = calculations.probability_collision_next_n_steps(star_calculations,0,.2,[.5,1,0,1,0.1,0.1,.1,.1],[0.5,0],0.25)
    plt.plot([1,1,1],[1,2,3],color="red", label="1 Time Step")
    plt.plot([1,1,1],[1,2,3],color="blue", label="2 Time Steps")
    plt.plot([1,1,1],[1,2,3],color="green", label="3 Time Steps")
    plt.plot([1,1,1],[1,2,3],color="yellow", label="4 Time Steps")
    plt.plot([1,1,1],[1,2,3],color="purple", label="5 Time Steps")
    #plt.plot(x,collisions_2,color="blue")
    #plt.plot(x,collisions_3,color="green")
    # plt.plot(x,collisions,color="red")
    plt.xlabel("Time Steps in Future")
    plt.ylabel("Probability Of Collision")
    plt.legend(loc = "upper left")
    plt.show()