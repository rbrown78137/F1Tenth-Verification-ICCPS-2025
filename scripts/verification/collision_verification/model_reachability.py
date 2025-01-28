import math
from math import cos, tan, atan
import numpy as np
import verification.collision_verification.collision_verification_constants as constants
# V = Car Velocity, s = steering angle, dt = model time duration
def circular_path_jacobian_A(raw_V, s, dt):
    # Clipping at low velocities to prevent long term motion of stationary objects
    V=raw_V
    if(V<constants.MINIMUM_VELOCITY_CLIPPING_VALUE):
        V=0
    B = atan(tan(s)/2)
    dtheta_xdtheta_y = -V * tan(s) * cos(B) / constants.WHEELBASE
    dtheta_ydtheta_x = V * tan(s) * cos(B) / constants.WHEELBASE
    A = np.array([
        [1, 0, V * dt, 0],
        [0, 1, 0, V * dt],
        [0, 0, 1, dtheta_xdtheta_y * dt],
        [0, 0, dtheta_ydtheta_x * dt, 1],
    ])
    return A

def two_car_A(V_pi, s_pi,V_omega,s_omega, dt):
    A_pi = circular_path_jacobian_A(V_pi,s_pi,dt)
    A_omega = circular_path_jacobian_A(V_omega,s_omega,dt)
    A = np.bmat([
			[A_pi,np.zeros((4,4))],
			[np.zeros((4,4)),A_omega]
	])
    return A