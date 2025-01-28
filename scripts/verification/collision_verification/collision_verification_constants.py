import math

WHEELBASE = 0.4 # m
PI_AVERAGE_STEERING_ANGLE_ERROR = 0.0174533 # 1 degree. Variable used to store average error in output steering angle when compared with command from control 
MAX_CAR_STEERING_ANGLE = 0.523599 # 40 degrees
MAX_CAR_CURVATURE = 1 / math.sqrt(WHEELBASE**2 *(1/4 + math.cos(MAX_CAR_STEERING_ANGLE)**2 / math.cos(MAX_CAR_STEERING_ANGLE)**2))
MINIMUM_VELOCITY_CLIPPING_VALUE = 0.10 #m/s. Minimum velocity for moving objects. Prevents long term drift of stationary objects
POSE_DIFFERENTIATION_DISTANCE = 0.02 # m. Distance at which the difference between two poses should be considered movement between cars and not noise due to pose prediction
PI_CAR_LENGTH = 0.55 # m
PI_CAR_WIDTH = 0.2 # m
OMEGA_CAR_LENGTH = 0.55 # m
OMEGA_CAR_WIDTH = 0.2 # m
REACHABILITY_DT = 0.25 # s
POSE_DT = 0.6 # s. Time between pose predictions used to estimate steering angle and velocity 
POSES_TO_AVERAGE_POSITION = 1
STEERING_AND_VELOCITY_PREDICTIONS_TO_AVERAGE = 4
MODEL_SUBTIME_STEPS = 10
K_STEPS = 5

# Variable To adjust odometry and perception input 
RECORDING_NUMBER = 8

LOG_TIMING_INFO = False