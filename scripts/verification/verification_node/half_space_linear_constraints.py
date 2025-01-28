import math
import numpy as np
import verification.verification_node.verification_constants as constants

# returns angle in 0<= angle < 2pi range
def wrapAngle(angle):
    while angle < 0:
        angle += 2 * math.pi
    while angle >= 2 * math.pi:
        angle -= 2 * math.pi
    return angle


# Coordinate location of the point that would be considered the top right before you rotate the car viewpoint
# angle_pi represents the rotation counterclockwise from the standard xz plane where the car faces in the positive z direction
def preRotationTopRight(angle_pi, length, width):
    x = (width/2) * math.cos(angle_pi) - (length/2) * math.sin(angle_pi)
    z = (length/2) * math.cos(angle_pi) + (width/2) * math.sin(angle_pi)
    return x, z


# Coordinate location of the point that would be considered the top left before you rotate the car viewpoint
# angle_pi represents the rotation counterclockwise from the standard xz plane where the car faces in the positive z direction
def preRotationTopLeft(angle_pi, length, width):
    x = -(width/2) * math.cos(angle_pi) - (length/2) * math.sin(angle_pi)
    z = (length/2) * math.cos(angle_pi) - (width/2) * math.sin(angle_pi)
    return x, z


# Coordinate location of the point that would be considered the bottom left before you rotate the car viewpoint
# angle_pi represents the rotation counterclockwise from the standard xz plane where the car faces in the positive z direction
def preRotationBottomRight(angle_pi, length, width):
    x = (width/2) * math.cos(angle_pi) + (length/2) * math.sin(angle_pi)
    z = -(length/2) * math.cos(angle_pi) + (width/2) * math.sin(angle_pi)
    return x, z


# Coordinate location of the point that would be considered the bottom right before you rotate the car viewpoint
# angle_pi represents the rotation counterclockwise from the standard xz plane where the car faces in the positive z direction
def preRotationBottomLeft(angle_pi, length, width):
    x = -(width/2) * math.cos(angle_pi) + (length/2) * math.sin(angle_pi)
    z = -(length/2) * math.cos(angle_pi) - (width/2) * math.sin(angle_pi)
    return x, z


# Corner in Positive Z direction or top right corner if car faces directly forward in positive z
def getTopCorner(angle_pi, length, width):
    if wrapAngle(angle_pi) < math.pi / 2:
        return preRotationTopRight(angle_pi,length,width)
    if wrapAngle(angle_pi) < math.pi:
        return preRotationBottomRight(angle_pi, length, width)
    if wrapAngle(angle_pi) < 3 * math.pi / 2:
        return preRotationBottomLeft(angle_pi, length, width)
    return preRotationTopLeft(angle_pi, length, width)


# Corner in Positive X direction or bottom right corner if car faces directly forward in positive z
def getRightCorner(angle_pi, length, width):
    if wrapAngle(angle_pi) < math.pi / 2:
        return preRotationBottomRight(angle_pi, length, width)
    if wrapAngle(angle_pi) < math.pi:
        return preRotationBottomLeft(angle_pi, length, width)
    if wrapAngle(angle_pi) < 3 * math.pi / 2:
        return preRotationTopLeft(angle_pi, length, width)
    return preRotationTopRight(angle_pi, length, width)


# Corner in Negative Z direction or bottom left corner if car faces directly forward in positive z
def getBottomCorner(angle_pi, length, width):
    if wrapAngle(angle_pi) < math.pi / 2:
        return preRotationBottomLeft(angle_pi, length, width)
    if wrapAngle(angle_pi) < math.pi:
        return preRotationTopLeft(angle_pi, length, width)
    if wrapAngle(angle_pi) < 3 * math.pi / 2:
        return preRotationTopRight(angle_pi, length, width)
    return preRotationBottomRight(angle_pi, length, width)


# Corner in Negative X direction or top left corner if car faces directly forward in positive z
def getLeftCorner(angle_pi, length, width):
    if wrapAngle(angle_pi) < math.pi / 2:
        return preRotationTopLeft(angle_pi, length, width)
    if wrapAngle(angle_pi) < math.pi:
        return preRotationTopRight(angle_pi, length, width)
    if wrapAngle(angle_pi) < 3 * math.pi / 2:
        return preRotationBottomRight(angle_pi, length, width)
    return preRotationBottomLeft(angle_pi, length, width)


# returns X_pi coefficient, Z_pi coefficient, X_omega coefficient, Z_omega coefficient, and constant
# in form a * X_pi + b * Z_pi + c * X_omega + d * Z_omegea <= e
def get_line_equation_from_two_points(x1, z1, x2, z2):
    a = z2 - z1
    b = -(x2 - x1)
    c = -(z2 - z1)
    d = x2 - x1
    e = z1 * (x2 - x1) - x1 * (z2 - z1)
    return a, b, c, d, e


# pi represents car doing calculations
# omega represents other car on road
def get_linear_inequalities_from_constants(angle_pi, angle_omega, length_pi=constants.pi_car_length, width_pi=constants.pi_car_width, length_omega=constants.omega_car_length, width_omega=constants.omega_car_width):
    # Get X,Z for corners of each car
    x_top_pi, z_top_pi = getTopCorner(angle_pi, length_pi, width_pi)
    x_left_pi, z_left_pi = getLeftCorner(angle_pi, length_pi, width_pi)
    x_right_pi, z_right_pi = getRightCorner(angle_pi, length_pi, width_pi)
    x_bottom_pi, z_bottom_pi = getBottomCorner(angle_pi, length_pi, width_pi)
    x_top_omega, z_top_omega = getTopCorner(angle_omega, length_omega, width_omega)
    x_left_omega, z_left_omega = getLeftCorner(angle_omega, length_omega, width_omega)
    x_right_omega, z_right_omgea = getRightCorner(angle_omega, length_omega, width_omega)
    x_bottom_omega, z_bottom_omega = getBottomCorner(angle_omega, length_omega, width_omega)

    # Get X coordinate of corners of collision octagon
    x_bounding_box_corner_1 = x_top_pi - x_bottom_omega
    x_bounding_box_corner_2 = x_top_pi - x_left_omega
    x_bounding_box_corner_3 = x_right_pi - x_left_omega
    x_bounding_box_corner_4 = x_right_pi - x_top_omega
    x_bounding_box_corner_5 = x_bottom_pi - x_top_omega
    x_bounding_box_corner_6 = x_bottom_pi - x_right_omega
    x_bounding_box_corner_7 = x_left_pi - x_right_omega
    x_bounding_box_corner_8 = x_left_pi - x_bottom_omega

    # Get Z coordinate of corners of collision octagon
    z_bounding_box_corner_1 = z_top_pi - z_bottom_omega
    z_bounding_box_corner_2 = z_top_pi - z_left_omega
    z_bounding_box_corner_3 = z_right_pi - z_left_omega
    z_bounding_box_corner_4 = z_right_pi - z_top_omega
    z_bounding_box_corner_5 = z_bottom_pi - z_top_omega
    z_bounding_box_corner_6 = z_bottom_pi - z_right_omgea
    z_bounding_box_corner_7 = z_left_pi - z_right_omgea
    z_bounding_box_corner_8 = z_left_pi - z_bottom_omega

    # Create inequalities by finding line between two points
    inequalities = [get_line_equation_from_two_points(x_bounding_box_corner_1, z_bounding_box_corner_1, x_bounding_box_corner_2, z_bounding_box_corner_2),
                    get_line_equation_from_two_points(x_bounding_box_corner_2, z_bounding_box_corner_2, x_bounding_box_corner_3, z_bounding_box_corner_3),
                    get_line_equation_from_two_points(x_bounding_box_corner_3, z_bounding_box_corner_3, x_bounding_box_corner_4, z_bounding_box_corner_4),
                    get_line_equation_from_two_points(x_bounding_box_corner_4, z_bounding_box_corner_4, x_bounding_box_corner_5, z_bounding_box_corner_5),
                    get_line_equation_from_two_points(x_bounding_box_corner_5, z_bounding_box_corner_5, x_bounding_box_corner_6, z_bounding_box_corner_6),
                    get_line_equation_from_two_points(x_bounding_box_corner_6, z_bounding_box_corner_6, x_bounding_box_corner_7, z_bounding_box_corner_7),
                    get_line_equation_from_two_points(x_bounding_box_corner_7, z_bounding_box_corner_7, x_bounding_box_corner_8, z_bounding_box_corner_8),
                    get_line_equation_from_two_points(x_bounding_box_corner_8, z_bounding_box_corner_8, x_bounding_box_corner_1, z_bounding_box_corner_1)]
    return inequalities

def simple_2_car_predicate(angle_pi, angle_omega):
    contstraints = get_linear_inequalities_from_constants(angle_pi, angle_omega)
    C = np.zeros((0,10))
    d = np.zeros((0,1))
    for constraint in contstraints:
        new_C_row = np.array([constraint[0],constraint[1],0,0,0,0,constraint[2],constraint[3],0,0])
        new_C_row = np.expand_dims(new_C_row,axis=0)
        new_d_entry = np.array([[constraint[4]]])
        C = np.concatenate([C,new_C_row],axis=0)
        d = np.concatenate([d,new_d_entry],axis=0)
    return C,d

# if __name__ == "__main__":
#     test = get_linear_inequalities_from_constants(0, math.pi/4, 1, 1, 1, 1)
#     please_work = 1