import math

nav_drive_topic = "/vesc/low_level/ackermann_cmd_mux/output"
# nav_drive_topic = "/vesc/high_level/ackermann_cmd_mux/input/nav_0"

pi_car_length = 0.55
pi_car_width = 0.2
omega_car_length = 0.55
omega_car_width = 0.2

number_worker_processes = 20
dt = 0.25
future_star_calculations = 5
velocity_estimate = 0.3
camera_rotation = 0
def wrapAngle(angle):
    while angle < 0:
        angle += 2 * math.pi
    while angle >= 2 * math.pi:
        angle -= 2 * math.pi
    return angle