#!/usr/bin/env python
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

        self.ranges = {"red":(np.array(UPPER_RED),np.array(LOWER_RED)),"green":(np.array(UPPER_GREEN),np.array(LOWER_GREEN))}        
        
        self.kernel = np.ones((5,5), np.uint8) # np.array(MATT,dtype=np.uint8)


    def sliceImage(self,img):
        return img[:int(img.shape[1]*0.6),:,:]


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
        width = int(img.shape[0]*self.img_scale_factor/100)
        height = int(img.shape[1]*self.img_scale_factor/100)


        img = cv2.resize(img,(height,width))
        img = cv2.rotate(img,cv2.ROTATE_180)
        img =  img[:175,:,:]
        img =cv2.GaussianBlur(img,(7,7),0)

        ##########################################################################################################
        
        return img

    def extractRedPixels(self, img):
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
        upperRed = np.array([15,255,255])
        lowerRed = np.array([0,88,88])

        upperRed2 = np.array([180,255,255])
        lowerRed2 = np.array([170,88,88])

        hsv = cv2.cvtColor(img.copy(),cv2.COLOR_BGR2HSV)


        filtered = cv2.inRange(hsv,lowerb=lowerRed,upperb=upperRed)
        filtered2 = cv2.inRange(hsv,lowerb=lowerRed2,upperb=upperRed2)

        masked = cv2.bitwise_and(img,img,mask=filtered+filtered2)


        ##########################################################################################################
        
        return masked

    def extractGreenPixels(self, img):
        # Method used to extract the green pixels from an image, the image processing must be divided in:
        # 1 - Convert the image to the HSV color space.
        # 2 - Define adequate HSV threshold values for the color filtering.
        # 3 - Use the required OpenCV function to obtain image mask required to filter the pixels based on the defined color threshold values.
        # 3 - Use the required OpenCV function to apply the obtained masks to the image.
        # 4 - Return the final processed image
        
        ##########################################################################################################
        # TODO: Complete the class method definition using the previous description
        ##########################################################################################################
        
        # Your code here...
        upperRed = np.array([65,255,255])
        lowerRed = np.array([45,180,88])        
        hsv = cv2.cvtColor(img.copy(),cv2.COLOR_BGR2HSV)
        filtered = cv2.inRange(hsv,lowerb=lowerRed,upperb=upperRed)
        masked = cv2.bitwise_and(img,img,mask=filtered)


        ##########################################################################################################
        
        return masked

    def binarizeImage(self, masked,erotionIters=5,dilationIters = 5):
        # Method used to binarize a colored input image, the image processing must be divided in:
        # 1 - Convert the image to a grayscale color space.
        # 2 - Use the requierd OpenCV function to binarize the image selecting an adequate theshold value.
        # 3 - Use the required OpenCV functions and kernels, to erode or dilate the binarized image and reduce non desired or noisy image components.
        # 4 - Return the final processed image

        ##########################################################################################################
        # TODO: Complete the class method definition using the previous description
        ##########################################################################################################

        # Your code here...
        gray = cv2.cvtColor(masked,cv2.COLOR_BGR2GRAY)
        threshold,thresh = cv2.threshold(gray,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
        erotion = cv2.erode(src=thresh,kernel=self.kernel ,iterations=erotionIters)
        dilation = cv2.dilate(src=erotion,kernel=self.kernel ,iterations=dilationIters)

        ##########################################################################################################
        
        return dilation

    def extractBlobs(self, img, color):
        # Method used to extract blobs form a binarized input image, the image processing must be divided in:
        # 1 - Use the requierd OpenCV function to instantiate a Blob Detector Parameters object.
        # 2 - Modify the minimum blob area an circularity parameters to better detect the desired blobs
        # 3 - Use the requierd OpenCV function to create a Blob Detector passing the defined Parameters object.
        # 4 - Extract the image blob keypoints using the detector.
        # 5 - Create a full black image with the same shape as your input image.
        # 6 - Draw the extracted blob keypoints on the black image, the keypoints must be drawn with the color specified by the method's color argument input.
        # 7 - Assing a boolean value to the 'found' variable, marking wheter an adequate blob was detected on the image or not.
        # 8 - Return both the boolean value and the image with the blob keypoints drawn.

        found = False

        ##########################################################################################################
        # TODO: Complete the class method definition using the previous description
        ##########################################################################################################

        # Your code here...
        params = cv2.SimpleBlobDetector_Params()
        
        params.filterByCircularity = True
        params.minCircularity = 0.6
        params.filterByArea = True
        params.minArea = 50
        params.maxArea = 5000
        
        detector = cv2.SimpleBlobDetector_create(params)
        keypoints = detector.detect(img)
        m = np.zeros(img.shape,dtype = np.uint8)
        black = cv2.merge([m,m,m])
        blobs = cv2.drawKeypoints(black,keypoints,np.array([]),color,cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        found = False if len(keypoints)  == 0 else True

        ##########################################################################################################
        
        return found, blobs

    def run(self):
        # Main loop
        while not rospy.is_shutdown():
            if self.image is None: 
                self.rate.sleep()
                continue

            src_frame = self.image

            preprocessed_image = self.preprocessImage(src_frame)
        
            red_pixel_image = self.extractRedPixels(preprocessed_image)
            green_pixel_image = self.extractGreenPixels(preprocessed_image)

            red_pixel_binarized_image = self.binarizeImage(red_pixel_image,dilationIters=1,erotionIters=1)
            green_pixel_binarized_image = self.binarizeImage(green_pixel_image)


            red_pixel_blob_found, red_pixel_blob_detection = self.extractBlobs(red_pixel_binarized_image, (0, 0, 255))
            green_pixel_blob_found, green_pixel_blob_detection = self.extractBlobs(green_pixel_binarized_image, (0, 255, 0))

            filtered_image = red_pixel_blob_detection | green_pixel_blob_detection

            ##########################################################################################################
            # TODO: Use the adequate cv_bridge method and class attribute to convert the filtered image from OpenCV format to ROS Image message format
            ##########################################################################################################

            output = self.bridge.cv2_to_imgmsg(filtered_image,encoding="bgr8")
            output2 = self.bridge.cv2_to_imgmsg(red_pixel_binarized_image)
            output3 = self.bridge.cv2_to_imgmsg(green_pixel_binarized_image)
            output4 = self.bridge.cv2_to_imgmsg(preprocessed_image,encoding="bgr8")
            
            ##########################################################################################################

            self.image_pub.publish(output)

            self.red_image_pub.publish(output2)
            self.green_image_pub.publish(output3)

            self.preprocessed_image_pub.publish(output4)

            self.red_light_detected_pub.publish(red_pixel_blob_found)

            self.green_light_detected_pub.publish(green_pixel_blob_found)

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