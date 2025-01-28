import rospy
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
import verification.manual_control.manual_control_constants as constants
from sensor_msgs.msg import Image
import cv_bridge
import cv2 as cv
import numpy as np
import math
from pynput import keyboard
import math

#GLOBAL VARS FOR STEERING
INITIAL_LANE = 1 # -1 inner lane 1 outer lane 2 transitioning to outer lane -2 transitioning to inner lane
DIRECTION_OF_LINES = -1# was 0
MAX_ANGLE = 0.6
LAST_ANGLE = 0
LAST_YELLOW_SIDE = 0
# CONSTANTS FOR STEERING 
SINGLE_LANE_DETECTION_CENTERING_OFFSET = 0.5
OFFSET_CONSTANT = 1
MAX_OFFSET = 1
ANGLE_CONSTANT = 0.7
#GLOBAL VARS FOR LANE RECOGNITION
H_MIN = 24
H_MAX = 28
S_MIN = 80
S_MAX = 255
V_MIN = 150
V_MAX = 255

YELLOW_H_MIN = 24
YELLOW_H_MAX = 28
YELLOW_S_MIN = 80
YELLOW_S_MAX = 255
YELLOW_V_MIN = 150
YELLOW_V_MAX = 255

# #Real World

WHITE_H_MIN = 0
WHITE_H_MAX = 255
WHITE_S_MIN = 0
WHITE_S_MAX = 100
WHITE_V_MIN = 180
WHITE_V_MAX = 255

# Simulator
# WHITE_H_MIN = 0
# WHITE_H_MAX = 10
# WHITE_S_MIN = 0
# WHITE_S_MAX = 255
# WHITE_V_MIN = 240
# WHITE_V_MAX = 255

TOP_CROP = 400
LINE_LENGTH = 20
SPEED = 0

bridge = cv_bridge.CvBridge()
slider_window = 'HSV Slider'
display_window = "Image"
birds_eye_window = "Bird's Eye View"
BIRDS_EYE_VIEW_WINDOW_SIZE = 200
#cv.namedWindow(slider_window)
#cv.namedWindow(display_window)
#cv.namedWindow(birds_eye_window)

#def on_H_MIN_trackbar(val):
#    global H_MIN, H_MAX, S_MIN, S_MAX, V_MIN, V_MAX
#    H_MIN = val
#def on_H_MAX_trackbar(val):
#    global H_MIN, H_MAX, S_MIN, S_MAX, V_MIN, V_MAX
#    H_MAX = val
#def on_S_MIN_trackbar(val):
#    global H_MIN, H_MAX, S_MIN, S_MAX, V_MIN, V_MAX
#    S_MIN = val
#def on_S_MAX_trackbar(val):
#    global H_MIN, H_MAX, S_MIN, S_MAX, V_MIN, V_MAX
#    S_MAX = val
#def on_V_MIN_trackbar(val):
#    global H_MIN, H_MAX, S_MIN, S_MAX, V_MIN, V_MAX
#    V_MIN = val
#def on_V_MAX_trackbar(val):
#    global H_MIN, H_MAX, S_MIN, S_MAX, V_MIN, V_MAX
#    V_MAX = val

# CREATE TRACKBARS
#H_MIN_trackbar_name = 'H MIN x %d' % 255
#cv.createTrackbar(H_MIN_trackbar_name, slider_window , H_MIN, 255, on_H_MIN_trackbar)
#H_MAX_trackbar_name = 'H MAX x %d' % 255
#cv.createTrackbar(H_MAX_trackbar_name, slider_window , H_MAX, 255, on_H_MAX_trackbar)
#S_MIN_trackbar_name = 'S MIN x %d' % 255
#cv.createTrackbar(S_MIN_trackbar_name, slider_window , S_MIN, 255, on_S_MIN_trackbar)
#S_MAX_trackbar_name = 'S MAX x %d' % 255
#cv.createTrackbar(S_MAX_trackbar_name, slider_window , S_MAX, 255, on_S_MAX_trackbar)
#V_MIN_trackbar_name = 'V MIN x %d' % 255
#cv.createTrackbar(V_MIN_trackbar_name, slider_window , V_MIN, 255, on_V_MIN_trackbar)
#V_MAX_trackbar_name = 'V MAX x %d' % 255
#cv.createTrackbar(V_MAX_trackbar_name, slider_window , V_MAX, 255, on_V_MAX_trackbar)


def image_callback(data):
    global SPEED
    global WHITE_H_MIN, WHITE_H_MAX, WHITE_S_MIN, WHITE_S_MAX, WHITE_V_MIN, WHITE_V_MAX, YELLOW_H_MIN, YELLOW_H_MAX, YELLOW_S_MIN, YELLOW_S_MAX, YELLOW_V_MIN, YELLOW_V_MAX
    global H_MIN, H_MAX, S_MIN, S_MAX, V_MIN, V_MAX
    cv_image = bridge.imgmsg_to_cv2(data,desired_encoding='rgb8')
    display_image = cv.cvtColor(cv_image,cv.COLOR_RGB2BGR)
    cv_image = cv_image[TOP_CROP:480][...]
    hsv_image = cv.cvtColor(cv_image,cv.COLOR_RGB2HSV)
    range = cv.inRange(hsv_image,np.array([H_MIN,S_MIN,V_MIN]),np.array([H_MAX,S_MAX,V_MAX]))
    gaussian_blur = cv.GaussianBlur(range,(5,5),cv.BORDER_DEFAULT)
    canny_edge = cv.Canny(gaussian_blur,100,200)
    white_range = cv.inRange(hsv_image,np.array([WHITE_H_MIN,WHITE_S_MIN,WHITE_V_MIN]),np.array([WHITE_H_MAX,WHITE_S_MAX,WHITE_V_MAX]))
    white_gaussian_blur = cv.GaussianBlur(white_range,(5,5),cv.BORDER_DEFAULT)
    white_canny_edge = cv.Canny(white_gaussian_blur,100,200)
    yellow_range = cv.inRange(hsv_image,np.array([YELLOW_H_MIN,YELLOW_S_MIN,YELLOW_V_MIN]),np.array([YELLOW_H_MAX,YELLOW_S_MAX,YELLOW_V_MAX]))
    yellow_gaussian_blur = cv.GaussianBlur(yellow_range,(5,5),cv.BORDER_DEFAULT)
    yellow_canny_edge = cv.Canny(yellow_gaussian_blur,100,200)
    lsd = cv.createLineSegmentDetector(0)
    #Detect white lines in the image
    all_white_lines = lsd.detect(white_canny_edge)[0]
    white_lines = []
    if not all_white_lines is None:
        for line in all_white_lines:
            x0 = int(round(line[0][0]))
            y0 = int(round(line[0][1]))
            x1 = int(round(line[0][2]))
            y1 = int(round(line[0][3]))
            if(math.sqrt(pow((x1-x0),2)+pow((y1-y0),2)) > LINE_LENGTH):
                white_lines.append((x0,y0,x1,y1))
                display_image = cv.line(display_image, (int(x0),int(y0)+TOP_CROP), (int(x1), int(y1)+TOP_CROP), (0, 255, 0), 4)
    all_yellow_lines = lsd.detect(yellow_canny_edge)[0]
    yellow_lines = []
    if not all_yellow_lines is None:
        for line in all_yellow_lines:
            x0 = int(round(line[0][0]))
            y0 = int(round(line[0][1]))
            x1 = int(round(line[0][2]))
            y1 = int(round(line[0][3]))
            if(math.sqrt(pow((x1-x0),2)+pow((y1-y0),2)) > LINE_LENGTH):
                yellow_lines.append((x0,y0,x1,y1))
                display_image = cv.line(display_image, (int(x0),int(y0)+TOP_CROP), (int(x1), int(y1)+TOP_CROP), (0, 255, 0), 4)
    global_mat = np.ones((BIRDS_EYE_VIEW_WINDOW_SIZE, BIRDS_EYE_VIEW_WINDOW_SIZE, 3), dtype=np.uint8) * 255

    # Display global lines 
    global_yellow_lines = []
    for line in yellow_lines:
        start = translate_point(line[0],line[1])
        end = translate_point(line[2],line[3])
        global_yellow_lines.append([start[0],start[1],end[0],end[1]])
        display_x0 = start[0] /4 * BIRDS_EYE_VIEW_WINDOW_SIZE + BIRDS_EYE_VIEW_WINDOW_SIZE / 2
        display_y0 = -start[1] /4 * BIRDS_EYE_VIEW_WINDOW_SIZE + BIRDS_EYE_VIEW_WINDOW_SIZE
        display_x1 = end[0] /4 * BIRDS_EYE_VIEW_WINDOW_SIZE + BIRDS_EYE_VIEW_WINDOW_SIZE / 2
        display_y1 = -end[1] /4 * BIRDS_EYE_VIEW_WINDOW_SIZE + BIRDS_EYE_VIEW_WINDOW_SIZE
        display_x0 = max(0,min(BIRDS_EYE_VIEW_WINDOW_SIZE,display_x0))
        display_y0 = max(0,min(BIRDS_EYE_VIEW_WINDOW_SIZE,display_y0)) 
        display_x1 = max(0,min(BIRDS_EYE_VIEW_WINDOW_SIZE,display_x1))
        display_y1 = max(0,min(BIRDS_EYE_VIEW_WINDOW_SIZE,display_y1))
        global_mat = cv.line(global_mat, (int(display_x0),int(display_y0)), (int(display_x1), int(display_y1)), (0, 255, 0), 4)
    global_white_lines = []
# Display global lines 
    for line in white_lines:
        start = translate_point(line[0],line[1])
        end = translate_point(line[2],line[3])
        global_white_lines.append([start[0],start[1],end[0],end[1]])
        display_x0 = start[0] /4 * BIRDS_EYE_VIEW_WINDOW_SIZE + BIRDS_EYE_VIEW_WINDOW_SIZE / 2
        display_y0 = -start[1] /4 * BIRDS_EYE_VIEW_WINDOW_SIZE + BIRDS_EYE_VIEW_WINDOW_SIZE
        display_x1 = end[0] /4 * BIRDS_EYE_VIEW_WINDOW_SIZE + BIRDS_EYE_VIEW_WINDOW_SIZE / 2
        display_y1 = -end[1] /4 * BIRDS_EYE_VIEW_WINDOW_SIZE + BIRDS_EYE_VIEW_WINDOW_SIZE
        display_x0 = max(0,min(BIRDS_EYE_VIEW_WINDOW_SIZE,display_x0))
        display_y0 = max(0,min(BIRDS_EYE_VIEW_WINDOW_SIZE,display_y0)) 
        display_x1 = max(0,min(BIRDS_EYE_VIEW_WINDOW_SIZE,display_x1))
        display_y1 = max(0,min(BIRDS_EYE_VIEW_WINDOW_SIZE,display_y1))
        global_mat = cv.line(global_mat, (int(display_x0),int(display_y0)), (int(display_x1), int(display_y1)), (0, 255, 0), 4)
    
    #cv.imshow(winname=display_window, mat=display_image)
    #cv.waitKey(5)
    # Send Drive Command
    drive_command = AckermannDriveStamped()
    # drive_command.header.stamp = rospy.Time.now()
    # drive_command.header.frame_id = 'your_frame_here'
    steering_angle = get_steering_angle(global_white_lines,global_yellow_lines)
    drive_command.drive.steering_angle = steering_angle * -1
    drive_command.drive.speed = SPEED
    print(f"Steering Angle:{steering_angle * -1}")
    publisher.publish(drive_command)

def get_steering_angle(white_lines, yellow_lines):
    global INITIAL_LANE,DIRECTION_OF_LINES, LAST_ANGLE, LAST_YELLOW_SIDE
    average_yellow_steering = 0
    average_white_steering = 0
    average_x_coordinate_yellow = 0
    average_x_coordinate_white = 0
    final_angle = 0
    if len(yellow_lines)>0:
        total_angle = 0
        total_xcoordinate = 0
        lowest_x = 100
        highest_x = -100
        total_length = 0
        for line in yellow_lines:
            total_xcoordinate += (line[0] + line[2]) / 2
            if(line[0] > highest_x):
                highest_x = line[0]
            if(line[2] > highest_x):
                highest_x = line[2]
            if(line[0] < lowest_x):
                lowest_x = line[0]
            if(line[2] < lowest_x):
                lowest_x = line[2]
            line_length = math.sqrt(pow(line[0]-line[2],2)+pow(line[1]-line[3],2))
            total_length = total_length + line_length   
            total_angle += get_average_angle(line[0],line[1],line[2],line[3]) * line_length
        average_yellow_steering = total_angle / total_length
        diff = 0
        if(average_yellow_steering < 0):
            diff = (highest_x - lowest_x) /1.5
        if(average_yellow_steering > 0):
            diff = -(highest_x - lowest_x) /1.5
        average_x_coordinate_yellow = (total_xcoordinate / len(yellow_lines)) + diff

    if len(white_lines)>0:
        total_angle = 0
        total_xcoordinate = 0
        lowest_x = 100
        highest_x = -100
        total_length = 0
        for line in white_lines:
            total_xcoordinate += (line[0] + line[2]) / 2
            if(line[0] > highest_x):
                highest_x = line[0]
            if(line[2] > highest_x):
                highest_x = line[2]
            if(line[0] < lowest_x):
                lowest_x = line[0]
            if(line[2] < lowest_x):
                lowest_x = line[2]
            line_length = math.sqrt(pow(line[0]-line[2],2)+pow(line[1]-line[3],2))
            total_length = total_length + line_length   
            total_angle += get_average_angle(line[0],line[1],line[2],line[3]) * line_length
        average_white_steering = total_angle / total_length
        diff = 0
        if(average_white_steering < 0):
            diff = (highest_x - lowest_x) /1.5
        if(average_white_steering > 0):
            diff = -(highest_x - lowest_x) /1.5
        average_x_coordinate_white = (total_xcoordinate / len(white_lines)) + diff
    # SWITCH LANE LABEL IF POSSIBLE
    if INITIAL_LANE == -2:
        if len(yellow_lines)>0:
            INITIAL_LANE = -1
            if average_x_coordinate_yellow >0:
                LAST_YELLOW_SIDE = 1
            else:
                LAST_YELLOW_SIDE = -1
    if INITIAL_LANE == 2:
        if len(yellow_lines)>0:
            INITIAL_LANE = 1
            if average_x_coordinate_yellow >0:
                LAST_YELLOW_SIDE = 1
            else:
                LAST_YELLOW_SIDE = -1
    # SET FINAL ANGLE
    if(INITIAL_LANE == -1 and len(yellow_lines) > 0):
        if average_x_coordinate_yellow >0:
            LAST_YELLOW_SIDE = 1
        else:
            LAST_YELLOW_SIDE = -1
        relative_adjustment = average_x_coordinate_yellow + DIRECTION_OF_LINES * SINGLE_LANE_DETECTION_CENTERING_OFFSET
        final_angle = ANGLE_CONSTANT * min(MAX_ANGLE,max(-MAX_ANGLE,average_yellow_steering)) + min(MAX_OFFSET,max(-MAX_OFFSET,OFFSET_CONSTANT * relative_adjustment))
        LAST_ANGLE = final_angle
    elif(INITIAL_LANE == 1 and len(white_lines) > 0 and len(yellow_lines) == 0):
        relative_adjustment = average_x_coordinate_white + DIRECTION_OF_LINES * SINGLE_LANE_DETECTION_CENTERING_OFFSET
        final_angle = ANGLE_CONSTANT * min(MAX_ANGLE,max(-MAX_ANGLE,average_white_steering)) + min(MAX_OFFSET,max(-MAX_OFFSET,OFFSET_CONSTANT * relative_adjustment))
        LAST_ANGLE = final_angle

    elif(INITIAL_LANE == 1 and len(white_lines) > 0 and len(yellow_lines) > 0 and average_x_coordinate_white * DIRECTION_OF_LINES <0):
        if average_x_coordinate_yellow >0:
            LAST_YELLOW_SIDE = 1
        else:
            LAST_YELLOW_SIDE = -1
        relative_adjustment = average_x_coordinate_white + DIRECTION_OF_LINES * SINGLE_LANE_DETECTION_CENTERING_OFFSET
        final_angle = ANGLE_CONSTANT * min(MAX_ANGLE,max(-MAX_ANGLE,average_white_steering)) + min(MAX_OFFSET,max(-MAX_OFFSET,OFFSET_CONSTANT * relative_adjustment))
        LAST_ANGLE = final_angle
    # Proper LANE NOT DETECTED
    elif(INITIAL_LANE == -1 and len(yellow_lines) == 0):
        final_angle = MAX_ANGLE * LAST_YELLOW_SIDE
    elif(INITIAL_LANE == 1 and len(white_lines) == 0 and len(yellow_lines) == 0 ):
        final_angle = MAX_ANGLE * LAST_YELLOW_SIDE
    elif(INITIAL_LANE == 0 and (len(white_lines) == 0 or len(yellow_lines) == 0 )):
        final_angle = MAX_ANGLE * DIRECTION_OF_LINES
    elif(INITIAL_LANE==2):
        final_angle = MAX_ANGLE * LAST_YELLOW_SIDE
    elif(INITIAL_LANE==-2):
        final_angle = MAX_ANGLE * LAST_YELLOW_SIDE    
    # This line addresses overtaking additional scneario but may be troublesome
    else:
        final_angle = MAX_ANGLE * DIRECTION_OF_LINES
    # print(f"Initial Lane: {INITIAL_LANE}")
    # print(f"DIRECTION_OF_LINES: {DIRECTION_OF_LINES}")
    return min(MAX_ANGLE,max(-MAX_ANGLE,final_angle))
def get_average_angle(x0,y0,x1,y1):
    if(abs(y1-y0)>0.00001):
        slope = (x1-x0) / (y1-y0)
        if slope > 0:
            angle = math.pi/2 - math.asin(abs((y1-y0)) / math.sqrt(pow(x1-x0,2)+pow(y1-y0,2)))
            return angle
        else:
            angle = -(math.pi/2 - math.asin(abs((y1-y0)) / math.sqrt(pow(x1-x0,2)+pow(y1-y0,2))))
            return angle
    return 0
def translate_point(pixel_x,pixel_y):
    local_x = pixel_x - constants.horizontal_center
    local_y = pixel_y - constants.vertical_center
    view_frame_z = local_y / (constants.camera_height * constants.horizontal_focal_length)
    view_frame_x = (local_x / constants.horizontal_focal_length) * view_frame_z
    return view_frame_x, view_frame_z
    # view_frame_x = local_x - constants.camera_x_offset
    # view_frame_y = local_y - constants.camera_y_offset
    # view_frame_z = local_z - constants.camera_z_offset
    # if view_frame_z <= 0:
    #     return None
    # pixel_x = (view_frame_x / view_frame_z) * constants.horizontal_focal_length 
    # pixel_y = (view_frame_y / view_frame_z) * constants.horizontal_focal_length
    # pixel_x = pixel_x + constants.horizontal_center
    # pixel_y = -1 * pixel_y + constants.vertical_center
    # #clip to range
    # pixel_x = max(0,min(pixel_x,constants.camera_pixel_width))
    # pixel_y = max(0,min(pixel_y,constants.camera_pixel_height))
    # return pixel_x, pixel_y
def on_press(key):
    global INITIAL_LANE, SPEED
    print("Got Key INPUT")
    if key == keyboard.Key.esc:
        return False  # stop listener
    try:
        k = key.char  # single-char keys
    except:
        k = key.name  # other keys
    if k in ['s']:  # keys of interest
        print('SWITCHING LANES: ')
        if(SPEED <= 1):
            if INITIAL_LANE == 1:
                INITIAL_LANE = -2
            elif INITIAL_LANE == -1:
                INITIAL_LANE = 2
    if k in ['q']:  # keys of interest
        print("SLOW")
        SPEED = 0.2
    if k in ['w']:  # keys of interest
        print("SLOW")
        SPEED = 0.4
    if k in ['e']:  # keys of interest
        print("MED")
        SPEED = 0.5
    if k in ['r']:  # keys of interest
        print("FAST")
        SPEED = 0.75
    if k in ['t']:  # keys of interest
        print("FAST")
        SPEED = 1
    if k in ['p']:  # keys of interest
        print("STOP")
        SPEED = 0

if __name__ == '__main__':

    # signal.signal(signal.SIGINT,sigint_handler)
    rospy.init_node('verification_node')
    publisher = rospy.Publisher(constants.nav_drive_topic, AckermannDriveStamped, queue_size=10,)
    rospy.Subscriber(constants.camera_topic, Image, image_callback,queue_size=1000)
    keyboard_listener = keyboard.Listener(on_press=on_press)
    keyboard_listener.start()  
    cv.waitKey()
    rospy.spin()
