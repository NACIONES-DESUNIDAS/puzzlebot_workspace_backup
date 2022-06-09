#!/usr/bin/env python
import cv2
from cv2 import cvtColor
from cv2 import merge
from matplotlib.pyplot import draw
import numpy as np
import rospy, cv_bridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool



# ROS_PARAMS and ROS_TOPICS constants used to define the package's parameters defined in the ROS parameter server
ROS_PARAMS = '/puzzlebot_vision/traffic_signals/parameters'
ROS_TOPICS = '/puzzlebot_vision/traffic_signals/topics'



# Constants used as default parameter values if no definition is found in the parameter server
RATE   =  30
IMG_SCALE_FACTOR = 50
CAMERA_TOPIC = '/video_source/raw'
LOWER_RED  = [0,88,179]
UPPER_RED = [33, 255, 255]
LOWER_GREEN = [98,255,255]
UPPER_GREEN = [49, 39, 130]



# Erode Delay Kernel:
MATT = [[1/9,1/9,1/9],[1/9,1/9,1/9],[1/9,1/9,1/9]]


# ROS Topics used to publish the desired output messages

# image topics
ROS_PREPROCESSED_IMAGE_TOPIC = '/puzzlebot_vision/traffic_signals/preprocessed_image'
ROS_BLUE_SIGNAL_IMAGE = "/puzzlebot_vision/traffic_signals/blue_image"
ROS_BLUE_SIGNAL_BINARIZED = "/puzzlebot_vision/traffic_signals/blue_image_bin"

ROS_RED_SIGNAL_IMAGE = "/puzzlebot_vision/traffic_signals/red_image"
ROS_RED_SIGNAL_BINARIZED = "/puzzlebot_vision/traffic_signals/red_image_bin"


ROS_WHITE_SIGNAL_IMAGE = "/puzzlebot_vision/traffic_signals/white_image"
ROS_WHITE_SIGNAL_BINARIZED = "/puzzlebot_vision/traffic_signals/white_image_bin"


ROS_IMAGE_OUTPUT_TOPIC = '/puzzlebot_vision/traffic_signals/filtered_image'


# traffic signals topics
ROS_SIGNAL_DETECT_TOPIC = '/puzzlebot_vision/traffic_signals/signal_found'
#ROS_BLUE_SIGNAL_DETECT_TOPIC = '/puzzlebot_vision/traffic_signals/blue_signal_found'
#ROS_WHITE_SIGNAL_DETECT_TOPIC = '/puzzlebot_vision/traffic_signals/white_signal_found'



class Signal_Identifier:
    def __init__(self):
        # Initialize general execution constants from parameter server or from their default values
        pub_rate = 0
        if rospy.has_param(ROS_PARAMS + '/signal_detection_pub_rate'):
            pub_rate = rospy.get_param(ROS_PARAMS + '/signal_detection_pub_rate')
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
        if rospy.has_param(ROS_TOPICS + '/camera_topic'):
            camera_topic = rospy.get_param(ROS_TOPICS + '/camera_topic')
            rospy.loginfo("CAMERA TOPIC LOADED FROM PARAMETER SERVER: %s", camera_topic)
        else:
            camera_topic = "/video_source/raw"

        # Class attribute used to store the current camera image 
        self.image = None

        self.bridge = cv_bridge.CvBridge()

        # Subscribe to camera topic
        self.cameraSub = rospy.Subscriber(camera_topic, Image, self.image_callback)

        # publishers

        self.preprocessedImagePub = rospy.Publisher(ROS_PREPROCESSED_IMAGE_TOPIC,Image,queue_size=10)
        self.outputImagePub = rospy.Publisher(ROS_IMAGE_OUTPUT_TOPIC,Image,queue_size=10)

        self.redImagePub = rospy.Publisher(ROS_RED_SIGNAL_IMAGE,Image,queue_size=10)
        self.redBinImagePub = rospy.Publisher(ROS_RED_SIGNAL_BINARIZED,Image,queue_size=10)


        self.whiteImagePub = rospy.Publisher(ROS_WHITE_SIGNAL_IMAGE,Image,queue_size=10)
        self.whiteBinImagePub = rospy.Publisher(ROS_WHITE_SIGNAL_BINARIZED,Image,queue_size=10)


        self.blueImagePub = rospy.Publisher(ROS_BLUE_SIGNAL_IMAGE,Image,queue_size=10)
        self.blueBinImagePub = rospy.Publisher(ROS_BLUE_SIGNAL_BINARIZED,Image,queue_size=10)




        # detection flags for potential candidates
        self.signalDetectedPub = rospy.Publisher(ROS_SIGNAL_DETECT_TOPIC, Bool, queue_size = 10)
        #self.blueSignalDetectedPub = rospy.Publisher(ROS_BLUE_SIGNAL_DETECT_TOPIC, Bool, queue_size = 10)
        #self.whiteSignalDetectedPub = rospy.Publisher(ROS_WHITE_SIGNAL_DETECT_TOPIC, Bool, queue_size = 10)


        # Initialize a rospy node with the name 'puzzlebot_traffic_lights'.
        rospy.init_node('puzzlebot_traffic_signals')

        # Define the ROS node execution rate
        self.rate = rospy.Rate(pub_rate)

        self.kernel = np.array([[1/9,1/9,1/9],[1/9,1/9,1/9],[1/9,1/9,1/9]])



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
        resized = img.copy()
        img =cv2.GaussianBlur(img,(7,7),0)
        ##########################################################################################################
        return img,resized


    def equalizeSV(self,hsv):
        H,S,V = cv2.split(hsv)
        S = cv2.equalizeHist(S)
        V = cv2.equalizeHist(S)
        equalized = cv2.merge([H,S,V])
        return equalized


    def extractRedPixels(self, img):


        upperRed = np.array([10,255,255])
        lowerRed = np.array([0,150,196])

        upperRed2 = np.array([180,255,255])
        lowerRed2 = np.array([170,150,196])
        lowerRed2 = np.array([170,170,200])

        hsv = cv2.cvtColor(img.copy(),cv2.COLOR_BGR2HSV)
        hsv = self.equalizeSV(hsv)


        filtered = cv2.inRange(hsv,lowerb=lowerRed,upperb=upperRed)
        filtered2 = cv2.inRange(hsv,lowerb=lowerRed2,upperb=upperRed2)

        masked = cv2.bitwise_and(img,img,mask=filtered+filtered2)
        
        return masked






    def extractBluePixels(self, img):

        upperBlue = np.array([105,255,255])
        lowerBlue = np.array([98,240,240])        
        hsv = cv2.cvtColor(img.copy(),cv2.COLOR_BGR2HSV)
        hsv = self.equalizeSV(hsv)

        filtered = cv2.inRange(hsv,lowerb=lowerBlue,upperb=upperBlue)
        masked = cv2.bitwise_and(img,img,mask=filtered)


        
        return masked


    def extractWhitePixels(self, img):

        #upperWhite = np.array([0,50,255])
        #lowerWhite = np.array([0,0,0])
        upperWhite = np.array([255,50,255])
        lowerWhite = np.array([150,0,150])         
        hsv = cv2.cvtColor(img.copy(),cv2.COLOR_BGR2RGB)
        #hsv = self.equalizeSV(hsv)

        filtered = cv2.inRange(hsv,lowerb=lowerWhite,upperb=upperWhite)
        masked = cv2.bitwise_and(img,img,mask=filtered)


        
        return masked
    


    def binarizeImage(self, masked,itersErotion=5,itersDilation = 10,dims = 5):

        gray = cv2.cvtColor(masked,cv2.COLOR_BGR2GRAY)
        threshold,thresh = cv2.threshold(gray,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
        dilation = cv2.dilate(src=thresh,kernel=np.ones((dims,dims)) ,iterations=itersDilation)
        erotion = cv2.erode(src=dilation,kernel=np.ones((dims,dims)) ,iterations=itersErotion)

        
        return erotion




    def extractBlobs(self, img, color):

        found = False


        params = cv2.SimpleBlobDetector_Params()
        params.filterByCircularity = True
        params.minCircularity = 0.7
        params.filterByArea = True
        params.minArea = 1500
        params.maxArea = 60000
        detector = cv2.SimpleBlobDetector_create(params)
        keypoints = detector.detect(img)
        m = np.zeros(img.shape,dtype = np.uint8)
        black = cv2.merge([m,m,m])
        #black = cv2.drawKeypoints(black,keypoints,np.array([]),color,cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        found = False if len(keypoints)  == 0 else True
        if len(keypoints) > 0:
            for keypoint in keypoints:
                x = int(keypoint.pt[0])
                y = int(keypoint.pt[1])
                rad = int((keypoint.size/2)+10)
                black = cv2.circle(black, (x,y), rad,color , -1)
                #print(x,y,rad)

        return found, black

    def extractContours(self,img):
        CONTOUR_THRESHOLD = 50000
        #img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        retval, img = cv2.threshold(img, 225, 255, cv2.THRESH_BINARY_INV)
        (contours, _) = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
        #contours = [contour for contour in contours if cv2.contourArea(contour) > 1000 and cv2.contourArea(contour) < 20000 ]
        defContours = list()
        for contour in contours:
            aprox = cv2.approxPolyDP(contour,0.01*cv2.arcLength(contour,True),True)
            area = cv2.contourArea(contour)
            if len(aprox > 8) and area > 4000 and area < 6000:
                defContours.append(contour)

        return defContours


    def getAreaOfInterest(self,contours,drawImage):
        if len(contours) > 0:
            #contoursImage = cv2.drawContours(drawImage, contours, -1, (0,255,0), 3)
            contoursImage = drawImage.copy()
            for contour in contours:
                #contour = contours[0]
                x,y,w,h = cv2.boundingRect(contour)
                contoursImage = cv2.rectangle(contoursImage, (x,y), (x+w,y+h), (200,0,200),2)
            #contoursImage = drawImage[y: y + h, x: x + w]
            return contoursImage
        else: 
            return drawImage










    def run(self):
        while not rospy.is_shutdown():
            if self.image is None: 
                self.rate.sleep()
                continue

            src_frame = self.image
            preprocessed_image,resizedImage = self.preprocessImage(src_frame)


            red = self.extractRedPixels(preprocessed_image)
            redBin = self.binarizeImage(red,itersDilation=1,itersErotion=1,dims = 3)
            redFound,redBlobs = self.extractBlobs(redBin,(255, 255, 255))
            controursRed = self.extractContours(redBin)
            aoiRed = self.getAreaOfInterest(controursRed,resizedImage)







            blue = self.extractBluePixels(preprocessed_image)
            blueBin = self.binarizeImage(blue,itersErotion=5,itersDilation=0,dims =5)
            blueFound,blueBlobs = self.extractBlobs(blueBin,(255, 255, 255))
            controursBlue = self.extractContours(blueBin)
            aoiBlue = self.getAreaOfInterest(controursBlue,resizedImage)

            
            white = self.extractWhitePixels(preprocessed_image)
            whiteBin = self.extractWhitePixels(preprocessed_image)
            #whiteFound,whiteBlobs = self.extractBlobs(whiteBin,(255, 255, 255))
            #controursWhite = self.extractContours(whiteBlobs)
            #aoiWhite = self.getAreaOfInterest(controursWhite,resizedImage)
            whiteFound = False


            flagOutput = redFound | blueFound | whiteFound


            preprocessedOutput = self.bridge.cv2_to_imgmsg(preprocessed_image,encoding="bgr8")



            redOutput = self.bridge.cv2_to_imgmsg(aoiRed,encoding="bgr8")
            blueOutput = self.bridge.cv2_to_imgmsg(aoiBlue,encoding="bgr8")
            whiteOutput = self.bridge.cv2_to_imgmsg(white,encoding="bgr8")

            redBinOutput = self.bridge.cv2_to_imgmsg(redBin)
            blueBinOutput = self.bridge.cv2_to_imgmsg(blueBin)
            whiteBinOutput = self.bridge.cv2_to_imgmsg(whiteBin)


            self.preprocessedImagePub.publish(preprocessedOutput)


            self.redImagePub.publish(redOutput)
            self.blueImagePub.publish(blueOutput)
            self.whiteImagePub.publish(whiteOutput)

            self.redBinImagePub.publish(redBinOutput)
            self.whiteBinImagePub.publish(whiteBinOutput)
            self.blueBinImagePub.publish(blueBinOutput)

            # publish detected candidates flags


            self.signalDetectedPub.publish(flagOutput)
            if flagOutput:
                defOutput = redOutput
                if redFound:
                    defOutput = redOutput
                elif blueFound:
                    defOutput = blueOutput
                else:
                    defOutput = whiteOutput
                    
                self.outputImagePub.publish(defOutput)
                

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg,desired_encoding="bgr8")


if __name__ == '__main__':
    traffic_signal_detector = Signal_Identifier()
    try:
        traffic_signal_detector.run()
    except rospy.ROSInterruptException:
        pass
