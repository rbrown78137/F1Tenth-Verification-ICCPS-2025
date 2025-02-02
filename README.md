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