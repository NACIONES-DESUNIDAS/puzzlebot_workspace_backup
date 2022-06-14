#!/usr/bin/env python
from cmath import rect
import cv2
from cv2 import cvtColor
import numpy as np
import rospy, cv_bridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

# ROS_PARAMS and ROS_TOPICS constants used to define the package's parameters defined in the ROS parameter server
ROS_PARAMS = '/puzzlebot_vision/traffic_lights/parameters'
ROS_TOPICS = '/puzzlebot_vision/traffic_lights/topics'

# Constants used as default parameter values if no definition is found in the parameter server
RATE   =  30
IMG_SCALE_FACTOR = 100
CAMERA_TOPIC = '/video_source/raw'
LOWER_RED  = [0,88,179]
UPPER_RED = [33, 255, 255]
LOWER_GREEN = [98,255,255]
UPPER_GREEN = [49, 39, 130]

# Erode Delay Kernel:
MATT = [[1/9,1/9,1/9],[1/9,1/9,1/9],[1/9,1/9,1/9]]


# ROS Topics used to publish the desired output messages
ROS_PREPROCESSED_TOPIC = '/puzzlebot_vision/traffic_lights/preprocessed_image'
ROS_IMAGE_OUTPUT_TOPIC = '/puzzlebot_vision/traffic_lights/filtered_image'
ROS_RED_LIGHT_DETECT_TOPIC = '/puzzlebot_vision/traffic_lights/red_light'
ROS_GREEN_LIGHT_DETECT_TOPIC = '/puzzlebot_vision/traffic_lights/green_light'

class TrafficLightsDetector:
    def __init__(self):

        # Initialize general execution constants from parameter server or from their default values
        pub_rate = 0
        if rospy.has_param(ROS_PARAMS + '/vision_pub_rate'):
            pub_rate = rospy.get_param(ROS_PARAMS + '/vision_pub_rate')
            rospy.loginfo("PUB RATE LOADED FROM PARAMETER SERVER: %s", pub_rate)
        else:
            pub_rate = RATE        

        self.img_scale_factor = 0
        if rospy.has_param(ROS_PARAMS + '/img_scale_factor'):
            self.img_scale_factor = rospy.get_param(ROS_PARAMS + '/img_scale_factor')
            rospy.loginfo("IMAGE SCALE FACTOR LOADED FROM PARAMETER SERVER: %s", self.img_scale_factor)
        else:
            self.img_scale_factor = IMG_SCALE_FACTOR     

        camera_topic =  None
        if rospy.has_param(ROS_PARAMS + '/camera_topic'):
            camera_topic = rospy.get_param(ROS_TOPICS + '/camera_topic')
            rospy.loginfo("CAMERA TOPIC LOADED FROM PARAMETER SERVER: %s", camera_topic)
        else:
            camera_topic = CAMERA_TOPIC

        # Class attribute used to store the current camera image 
        self.image = None

        ##########################################################################################################
        # TODO: Assign the bridge class attribute to the required cv_bridge instance used to pass images between ROS and OpenCV formats
        ##########################################################################################################

        self.bridge = cv_bridge.CvBridge()

        ##########################################################################################################

        # Subscribe to the camera images topic
        rospy.Subscriber(camera_topic, Image, self.image_callback)

        # Initialize Publisher that will send Image messages with the node processed image output
        self.image_pub = rospy.Publisher(ROS_IMAGE_OUTPUT_TOPIC, Image, queue_size = 10)

        self.green_image_pub = rospy.Publisher("green_binarized", Image, queue_size = 10)

        self.red_image_pub = rospy.Publisher("red_binarized", Image, queue_size = 10)

        self.preprocessed_image_pub = rospy.Publisher(ROS_PREPROCESSED_TOPIC,Image,queue_size=10)

        # Initialize Publishers that will send Bool messages if a red or green traffic light is detected
        self.red_light_detected_pub = rospy.Publisher(ROS_RED_LIGHT_DETECT_TOPIC, Bool, queue_size = 10)
        self.green_light_detected_pub = rospy.Publisher(ROS_GREEN_LIGHT_DETECT_TOPIC, Bool, queue_size = 10)
        # Initialize a rospy node with the name 'puzzlebot_traffic_lights'.
        rospy.init_node('puzzlebot_traffic_lights')

        # Define the ROS node execution rate
        self.rate = rospy.Rate(pub_rate)

    def preprocessImage(self, img):
        # Method used to preprocess the node input image, the image processing must be divided in:
        # 1 - Resize the input image to a specified image scale factor.
        # 2 - Rotate the image if required.
        # 3 - Apply an adequate Gaussian Blur to the image, modify the filter kernel as required.
        # 4 - Return the final processed image

        ##########################################################################################################
        # TODO: Complete the class method definition using the previous description
        ##########################################################################################################

        # Your code here...
        height =  int(float(img.shape[0]) * (float(self.img_scale_factor)/100.00))
        width = int(float(img.shape[1]) * (float(self.img_scale_factor)/100.00))
        size = (width, height)
        img = cv2.resize(img, size)
        #img = cv2.rotate(img, cv2.ROTATE_180)
        img = cv2.GaussianBlur(img, (3, 3), 0)

        ##########################################################################################################
        return img

    def extractPixels(self, img):
        # Method used to extract the red pixels from an image, the image processing must be divided in:
        # 1 - Convert the image to the HSV color space.
        # 2 - Define adequate HSV threshold values for the color filtering.
        # 3 - Use the required OpenCV function to obtain image mask required to filter the pixels based on the defined color threshold values.
        # 3 - Use the required OpenCV function to apply the obtained masks to the image.
        # 4 - Return the final processed image

        ##########################################################################################################
        # TODO: Complete the class method definition using the previous description
        ##########################################################################################################

        # Your code here...
        #Divide image by GREEN and RED channels, in addition to applying threshold from 220 to 255...
        #channel.
        img_b, img_green, img_red = cv2.split(img)
        th, redPixels = cv2.threshold(img_red, 220, 255, 0)
        th, greenPixels = cv2.threshold(img_green, 220, 255, 0)
        ##########################################################################################################
        
        #Returning mask for each color.
        return redPixels, greenPixels

    def contours(self, img, mask):
        #Find contours to generate a list to do the BoundingRect
        cnt, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        rect = []
        e = 5
        imagec=img.copy()
        for c in cnt:
            x, y, w, h = cv2.boundingRect(c)
            if(w>0) and (h>0):
                cv2.rectangle(imagec, (x, y), (x+w, y+h), (255,0, 0), 2)
                rect.append(img[y:y+h, x:x+w])
        
        
        return rect, imagec

    #CHANEL-->
    # RED-->1
    #GREEN-->2
    def detectPixels(self, rect, channel):
        counterRED = 0
        counterGREEN = 0
        if len(rect) > 0:

            if(channel == 1):
                for c in rect:
                    
                    c = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)
                    # H, S, V
                    red_min1 = np.array([0, 70, 110]) 
                    red_max1 = np.array([15, 255, 255])

                    red_min2 = np.array([170, 70, 110]) 
                    red_max2 = np.array([180, 255, 255])

                    mask1 = cv2.inRange(c, red_min1, red_max1)
                    mask2 = cv2.inRange(c, red_min2, red_max2)
                    mask = mask1 + mask2
                    counterRED+=np.count_nonzero(mask)
                rospy.loginfo(counterRED)
                
                #Flags in Boolean
                if(counterRED > 3):
                    self.found_red = True
                else:
                    self.found_red = False

            
            
            elif(channel == 2):
                for c in rect:
                    c = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)
                    green_min = np.array([30, 70, 80]) # H, S, V
                    green_max = np.array([80, 255, 255])
                    #green_min = np.array([45, 180, 88]) # H, S, V
                    #green_max = np.array([65, 255, 255])

                    mask = cv2.inRange(c, green_min, green_max)
                    counterGREEN+=np.count_nonzero(mask)
                #rospy.loginfo(counterGREEN)
                 #Flags in Boolean
                if(counterGREEN < 30):
                    self.found_green = True
                else:
                    self.found_green = False


    def run(self):
        # Main loop
        while not rospy.is_shutdown():
            if self.image is None: 
                self.rate.sleep()
                continue

            src_frame = self.image

            preprocessed_image = self.preprocessImage(src_frame)
            red_pixel_image, green_pixel_image = self.extractPixels(preprocessed_image)
            rectRED, contoursRED = self.contours(preprocessed_image, red_pixel_image)
            rectGREEN, contoursGREEN = self.contours(preprocessed_image, green_pixel_image)
            self.detectPixels(rectRED, 1)
            self.detectPixels(rectGREEN, 2)



            ##########################################################################################################
            # TODO: Use the adequate cv_bridge method and class attribute to convert the filtered image from OpenCV format to ROS Image message format
            ##########################################################################################################

            output = self.bridge.cv2_to_imgmsg(contoursRED, encoding="bgr8")
            output2 = self.bridge.cv2_to_imgmsg(green_pixel_image)
            output3 = self.bridge.cv2_to_imgmsg(contoursGREEN, encoding="bgr8")
            output4 = self.bridge.cv2_to_imgmsg(red_pixel_image)
            
            ##########################################################################################################
            ############################################FLASGSSSSSSSS########################
            self.red_light_detected_pub.publish(self.found_red)
            self.green_light_detected_pub.publish(self.found_green)
            ########################################################################################
            self.red_image_pub.publish(output)
            self.green_image_pub.publish(output3)


            #ME FALTARON LOS OUTPUTS Tanto del green_pixel_image & red_pixel_image
            #self.preprocessed_image_pub.publish(output2)

          

            self.image = None

            self.rate.sleep()

    def image_callback(self, msg):
        # Subscriber callback function used to store the camera extracted images

        ##########################################################################################################
        # TODO: Use the adequate cv_bridge method and class attribute to convert the obtained camera Image message to OpenCV image format
        ##########################################################################################################

        self.image = self.bridge.imgmsg_to_cv2(msg,desired_encoding="bgr8")

        ##########################################################################################################
        

if __name__ == '__main__':
    traffic_lights_detector = TrafficLightsDetector()
    try:
        traffic_lights_detector.run()
    except rospy.ROSInterruptException:
        pass