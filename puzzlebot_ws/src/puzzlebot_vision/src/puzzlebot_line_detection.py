#!/usr/bin/env python
import cv2
import numpy as np
import rospy, cv_bridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats






# the place that we use to fetch stuff
ROS_PARAMS = '/puzzlebot_vision/line_detection/parameters'
ROS_TOPICS = '/puzzlebot_vision/line_detection/topics'

RATE   =  30
IMG_HEIGHT = 360
IMG_WIDTH = 480
CAMERA_TOPIC = '/video_source/raw'


OUTPUT_IMAGE_TOPIC = "/puzzlebot_vision/line_detection/edges_detection_image"
OUTPUT_PREPROCESSED_IMAGE_TOPIC = "/puzzlebot_vision/line_detection/preprosecced_image"
OUTPUT_CHECKPOINT_TOPIC = "/puzzlebot_vision/line_detection/controller_set_point"




class LineDetector:
    def __init__(self):

        # try to fetch the camera topic from the configuration file
        if rospy.has_param(ROS_TOPICS + '/camera_topic'):
            camera_topic = rospy.get_param(ROS_TOPICS + '/camera_topic')
            rospy.loginfo("CAMERA TOPIC LOADED FROM PARAMETER SERVER: %s", camera_topic)
        else:
            camera_topic = CAMERA_TOPIC

        # try to fetch the publication rate from the files


        pub_rate = 0
        if rospy.has_param(ROS_PARAMS + '/line_detection_pub_rate'):
            pub_rate = rospy.get_param(ROS_PARAMS + '/line_detection_pub_rate')
            rospy.loginfo("PUB RATE LOADED FROM PARAMETER SERVER: %s", pub_rate)
        else:
            pub_rate = RATE    






        # try to fetch image scale factors

        if rospy.has_param(ROS_PARAMS + '/image_width'):
            self.imgWidth = rospy.get_param(ROS_PARAMS + '/image_width')
            rospy.loginfo("LOADED IMAGE WIDTH SCALE FACTOR")
        else:
            self.imgWidth = IMG_WIDTH    

        if rospy.has_param(ROS_PARAMS + '/image_height'):
            self.imgHeight = rospy.get_param(ROS_PARAMS + '/image_height')
            rospy.loginfo("LOADED IMAGE HEIGHT SCALE FACTOR")
        else:
            self.imgHeight = IMG_HEIGHT   

        

        # instanciate openCv Connector
        self.bridge = cv_bridge.CvBridge()
        self.image = None

        # publishers
        self.preprocessedImagePub = rospy.Publisher(OUTPUT_PREPROCESSED_IMAGE_TOPIC,Image,queue_size=10) # preprocessed image
        self.edgesImagePub = rospy.Publisher(OUTPUT_IMAGE_TOPIC,Image,queue_size=10) # edge detection (debug)
        self.lineCheckpoint = rospy.Publisher(OUTPUT_CHECKPOINT_TOPIC,Image,queue_size=1) # reference
        self.verticalSumPub = rospy.Publisher("/vertical_sum",numpy_msg(Floats),queue_size=10)




        # subscribers 
        self.rawVideoSubscriber = rospy.Subscriber(camera_topic,Image,self.imageCallback)

        # Initialize a rospy node with the name 'puzzlebot_traffic_lights'.
        rospy.init_node('puzzlebot_line_detection')

        # Define the ROS node execution rate
        self.rate = rospy.Rate(pub_rate)


        # define edge detection filters:
        self.sobelY = np.array([[-1,-2,-1], 
                                [ 0, 0, 0], 
                                [ 1, 2, 1]])

        self.sobelX = np.array([[ -1, 0, 1], 
                                [ -2, 0, 2], 
                                [ -1, 0, 1]])     

        # median filter
        self.medianFilter = np.array([
                                    [1/9,1/9,1/9],
                                    [1/9,1/9,1/9],
                                    [1/9,1/9,1/9]],dtype=np.uint8)   

    def imageCallback(self,msg):
        self.image = self.bridge.imgmsg_to_cv2(msg,desired_encoding="bgr8")

    def imagePreprocessing(self,img):
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        gray = cv2.resize(gray,(480,360))
        gray =cv2.GaussianBlur(gray,(5,5),0)
        return gray

    def sliceImage(self,img):
        return img[int(self.imgHeight*0.80):,:]

    def sumVertically(self,img):
        sum = img.sum(axis = 0)
        sum = np.float32(sum)
        return sum


    def edgeDetection(self,img):
        """
        canny = cv2.Canny(img,120,240)
        retval, binary = cv2.threshold(canny, 127, 255, cv2.THRESH_BINARY)
        contours,hierchy = cv2.findContours(canny,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        contourImage = np.copy(img)
        contourImage = cv2.drawContours(contourImage,contours,-1,(0,0,255),2)
        """
        """
        filteredX = cv2.filter2D(img, -1, self.sobelX)
        filteredY = cv2.filter2D(img, -1, self.sobelY)
        overall = filteredX+filteredY
        retval, binary = cv2.threshold(overall, 10, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        erotion = cv2.erode(src=binary,kernel=self.medianFilter ,iterations=1)
        binarized = cv2.dilate(src=binary,kernel=self.medianFilter ,iterations=3)
        """
        
        retval, binary = cv2.threshold(img, 10, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        erotion = cv2.erode(src=binary,kernel=self.medianFilter ,iterations=1)
        dilation = cv2.dilate(src=binary,kernel=self.medianFilter ,iterations=4)
        binarized = self.createImageMask(img,0,retval)
        
        return binarized
        



    def createImageMask(self,image,lowerBound,upperBound):
        return  cv2.inRange(image, lowerBound, upperBound)


        

        

    def run(self):
        #blackLower = np.array([0])
        #blackHigher = np.array([20])
        while not rospy.is_shutdown():
            if self.image is None:
                self.rate.sleep()

            frame = self.image

            # preprocess image
            preprocessedImage = self.imagePreprocessing(frame)
            # slice image (region of interest)
            preprocessedImage = self.sliceImage(preprocessedImage)
            # sum columns vertically
            vertSum = self.sumVertically(preprocessedImage)
            # binarize
            binarized = self.edgeDetection(preprocessedImage)


            proprocessedOutput = self.bridge.cv2_to_imgmsg(preprocessedImage)
            otherOutput = self.bridge.cv2_to_imgmsg(binarized)

            self.preprocessedImagePub.publish(proprocessedOutput)
            self.edgesImagePub.publish(otherOutput)
            self.verticalSumPub.publish(vertSum)



if __name__ == '__main__':
    lineDetector = LineDetector()
    try:
        lineDetector.run()
    except rospy.ROSInterruptException:
        pass