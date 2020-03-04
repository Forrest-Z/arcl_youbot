import rospy
import pyrealsense2 as rs
import cv2   
import numpy as np
import functools
import time
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Int8MultiArray
import signal
import sys

UNLOAD_HEIGHT = [0.17, 0.085, 0.01]
LOAD_HEIGHT = [0.2, 0.115, 0.03]
ZONE = [Pose(Point(0.315323382616, -1.01717340946, 0.103996619582), Quaternion(0.500282824039, -0.499866634607, 0.498632192612, -0.50121486187)),
        Pose(Point(-0.139539673924, -1.01989018917, 0.104218177497), Quaternion(0.498746186495, -0.503944218159, 0.49566963315, -0.5016015172)),
        Pose(Point(-0.593142211437, -1.02366733551, 0.106185980141), Quaternion(0.504684686661, -0.499624967575, 0.498205423355, -0.497453123331)),
        Pose(Point(-1.04596865177, -1.02525186539, 0.108994059265), Quaternion(0.504900157452, -0.500990092754, 0.496651381254, -0.497415482998))]
WIDTH = 640
HEIGHT = 480
NUM_ZONE = len(ZONE)
TOP = len(LOAD_HEIGHT)
EMPTY = 0
RED = 1
GREEN = 2
BLUE = 3
COLOR = [RED, GREEN, BLUE]      # len(COLOR) should be equal to NUM_ZONE
CV_MAX_DISTANCE = 2             # removing the background of objects more than
CV_MIN_AREA = 300               # valid object with area


class Box():

    def __init__(self, color, coordinate):
        self.color = color
        self.x1 = coordinate[0]
        self.y1 = coordinate[1]
        self.x2 = coordinate[2]
        self.y2 = coordinate[3]

    def __str__(self):
        return str(self.color) + " " + str(self.x1) + " " + str(self.y1) + " " + str(self.x2) + " " + str(self.y2)

    def __repr__(self):
        return str(self)

def compare_x(box1, box2):
    return box1.x1 -box2.x1

def compare_y(box1, box2):
    return box1.y1 -box2.y1

def get_cv_info(image_pub):
    # Create a pipeline
    pipeline = rs.pipeline()

    #Create a config and configure the pipeline to stream
    #  different resolutions of color and depth streams
    config = rs.config()
    # config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, 30)
    # Start streaming
    profile = pipeline.start(config)

    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print("Depth Scale is: " , depth_scale)

    # We will be removing the background of objects more than
    #  clipping_distance_in_meters meters away
    clipping_distance_in_meters = CV_MAX_DISTANCE # meter
    clipping_distance = clipping_distance_in_meters / depth_scale

    # Create an align object
    # rs.align allows us to perform alignment of depth frames to others frames
    # The "align_to" is the stream type to which we plan to align depth frames.
    align_to = rs.stream.color
    align = rs.align(align_to)

    start_time = time.time()
    msg = Int8MultiArray()
    r = rospy.Rate(10)

    try:
        while not rospy.is_shutdown():
            r.sleep()

            # Get frameset of color and depth
            frames = pipeline.wait_for_frames()

            # Align the depth frame to color frame
            aligned_frames = align.process(frames)

            # Get aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
            color_frame = aligned_frames.get_color_frame()

            # Validate that both frames are valid
            if not aligned_depth_frame or not color_frame:
                continue

            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Remove background - Set pixels further than clipping_distance to grey
            grey_color = 153
            depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
            bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)

            # start color detection
            # img = cv2.flip(bg_removed, -1)
            img = bg_removed
                
            #converting frame(img i.e BGR) to HSV (hue-saturation-value)
            hsv=cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            #definig the range of red color
            red_lower=np.array([0, 150, 100],np.uint8)
            red_upper=np.array([16, 236, 215],np.uint8)
            
            #defining the Range of green color
            green_lower=np.array([61, 105,  63],np.uint8)
            green_upper=np.array([81, 255, 174],np.uint8)

            #defining the Range of Blue color
            blue_lower=np.array([95, 203, 26],np.uint8)
            blue_upper=np.array([114, 255, 186],np.uint8)

            #finding the range of red,blue and green color in the image
            red=cv2.inRange(hsv, red_lower, red_upper)
            green=cv2.inRange(hsv,green_lower,green_upper)
            blue=cv2.inRange(hsv,blue_lower,blue_upper)

            #Morphological transformation, Dilation  	
            kernel = np.ones((5 ,5), "uint8")

            red = cv2.morphologyEx(red, cv2.MORPH_CLOSE, kernel, iterations=1)
            res_red = cv2.bitwise_and(img, img, mask = red)

            green = cv2.morphologyEx(green, cv2.MORPH_CLOSE, kernel, iterations=1)
            res_green = cv2.bitwise_and(img, img, mask = green)   

            blue = cv2.morphologyEx(blue, cv2.MORPH_CLOSE, kernel, iterations=1)
            res_blue = cv2.bitwise_and(img, img, mask = blue) 

            box_list = []

            #Tracking the Red Color
            (_,contours,hierarchy)=cv2.findContours(red,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            for pic, contour in enumerate(contours):
                area = cv2.contourArea(contour)
                if(area > CV_MIN_AREA):
                    x,y,w,h = cv2.boundingRect(contour)	
                    img = cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),2)
                    cv2.putText(img,"RED",(x,y),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255))
                    box_list.append(Box(RED, (x, y, x+w, y+h)))

            #Tracking the green Color
            (_,contours,hierarchy)=cv2.findContours(green,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            for pic, contour in enumerate(contours):
                area = cv2.contourArea(contour)
                if(area > CV_MIN_AREA):
                    x,y,w,h = cv2.boundingRect(contour)	
                    img = cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
                    cv2.putText(img,"GREEN",(x,y),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0))  
                    box_list.append(Box(GREEN, (x, y, x+w, y+h)))

            #Tracking the Blue Color
            (_,contours,hierarchy)=cv2.findContours(blue,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            for pic, contour in enumerate(contours):
                area = cv2.contourArea(contour)
                if(area > CV_MIN_AREA):
                    x,y,w,h = cv2.boundingRect(contour)	
                    img = cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
                    cv2.putText(img,"BLUE",(x,y),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0))
                    box_list.append(Box(BLUE, (x, y, x+w, y+h)))
                    
            # cv2.imshow("Redcolour",red)        
            # cv2.imshow("red",res)
            # cv2.imshow("Color Tracking",img)
            # key = cv2.waitKey(1)
            # if key & 0xFF == ord('s'):
            #     cv2.imwrite("test.jpg", img)
            #     cv2.destroyAllWindows()
            #     break
            # if key & 0xFF == ord('q') or key == 27:
            #     cv2.destroyAllWindows()
            #     break

            if time.time() - start_time > 1:
                print(box_list)

                first_place = []
                second_place = []
                third_place = []
                for b in box_list:
                    if b.x2 < WIDTH / 2:
                        first_place.append(b)
                    if b.x1 < WIDTH / 2 and b.x2 > WIDTH / 2:
                        second_place.append(b)
                    if b.x1 > WIDTH / 2:
                        third_place.append(b)
                
                first_place = sorted(first_place, key=functools.cmp_to_key(compare_y))
                second_place = sorted(second_place, key=functools.cmp_to_key(compare_y))
                third_place = sorted(third_place, key=functools.cmp_to_key(compare_y))
                box_places = [first_place, second_place, third_place]

                box_output = []
                for i in range(len(box_places)):
                    boxes = box_places[i]
                    num_empty = TOP - len(boxes)
                    temp = [EMPTY] * num_empty
                    box_output.extend(temp)
                    for b in boxes:
                        box_output.append(b.color)
                print(box_output)
                # cv2.imwrite("test.jpg", img)
                msg.data = box_output
                image_pub.publish(msg)
            
    finally:
        cv2.destroyAllWindows()
        pipeline.stop()

def ctrl_c_handler(signal, frame):
    """Gracefully quit the infrared_pub node"""
    print "\nCaught ctrl-c! Stopping node."
    sys.exit()

if __name__ == "__main__":
    rospy.init_node("image_pub")
    image_pub = rospy.Publisher('/image', Int8MultiArray, queue_size=1)
    signal.signal(signal.SIGINT, ctrl_c_handler)

    get_cv_info(image_pub)
