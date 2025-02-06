# f1tenth_verification

This folder contains the core algorithm for the paper Perception-based Quantitative Runtime Verification
for Learning-enabled Cyber-Physical Systems as well as the majority of the scripts used in the figures and tables to evaluate the performance of our algorithms.

This entire folder functions as a ROS package which can be added to any ros workspace.

The core algorithm is located in scripts/verification/collision_verification.
collision_probability.py contains the most high level implementation of this algorithm as a whole with the function probstar_next_k_time_steps.

The entire collsion verification algorithm can be run at runtime with collision_verification_ros_node.py and the roslaunch script on_start.launch. To run this file however, there must be a valid camera, and a running object deteciton node.

To run the object detection and pose detection pipeline, run object_tracking_node.launch or scripts/verification/object_tracking_node/object_tracking_node.py

To recreate the results from each figure in the paper, see the directory scripts/verification/paper_figure_scripts
All of these python files can be run a is and should produce similar or identical results based on the configuration of the host computer.
When running the paper figure scripts, run them from the F1Tenth-Verification-ICCPS-2025 so that the .pkl files are able to find the correct path.
For example:
python3 scripts/verification/paper_figure_scripts/figure_5_path_prediction_comparison.py

SEE THE ATTACHED DOCKER FILE FOR ANY ISSUES RELATED TO INSTALLATION

ASSUMING DOCKER IS INSTALLED, THE BELOW EXAMPLE SHOWS HOW TO RUN ONE OF THE PYTHON SCRIPTS IN FULL FROM DOCKER

docker build -t f1tenth_verification .
./docker_share_x11.sh
cd home/catkin_ws/src/F1Tenth-Verification-ICCPS-2025
python3 scripts/verification/paper_figure_scripts/figure_5_path_prediction_comparison.py
python3 scripts/verification/paper_figure_scripts/figure_6_3d_danger_area.py
python3 scripts/verification/paper_figure_scripts/figure_6_prob_collision_graph.py
python3 scripts/verification/paper_figure_scripts/figure_7_different_braking_forces.py
python3 scripts/verification/paper_figure_scripts/figure_7_different_steering_angles.py
python3 scripts/verification/paper_figure_scripts/table_1_accuracy.py
python3 scripts/verification/paper_figure_scripts/table_3_probstar_timing.py
python3 scripts/verification/paper_figure_scripts/table_4_probstar.py