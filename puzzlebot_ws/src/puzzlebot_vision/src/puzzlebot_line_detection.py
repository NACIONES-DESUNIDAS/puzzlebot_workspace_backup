#!/usr/bin/env python
import cv2
import numpy as np
import rospy, cv_bridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray, Float32MultiArray

# the place that we use to fetch stuff
ROS_PARAMS = '/puzzlebot_vision/line_detection/parameters'
ROS_TOPICS = '/puzzlebot_vision/line_detection/topics'

RATE   =  30
IMG_HEIGHT = 360
IMG_WIDTH = 480
CAMERA_TOPIC = '/video_source/raw'





OUTPUT_IMAGE_TOPIC = "/puzzlebot_vision/line_detection/edges_detection_image"
OUTPUT_PREPROCESSED_IMAGE_TOPIC = "/puzzlebot_vision/line_detection/preprocessed_image"
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
        #self.verticalSumPub = rospy.Publisher("/vertical_sum",numpy_msg(Floats),queue_size=10)
        self.verticalSumPub = rospy.Publisher("/vertical_sum",Int32MultiArray,queue_size=10)
        self.leftEdgePublisher = rospy.Publisher('/leftEdge', Float32MultiArray, queue_size = 10)
        self.rightEdgePublisher = rospy.Publisher('/rightEdge', Float32MultiArray, queue_size = 10)

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
                                    [1,1,1],
                                    [1,1,1],
                                    [1,1,1]],dtype=np.uint8)   

    def imageCallback(self,msg):
        self.image = self.bridge.imgmsg_to_cv2(msg,desired_encoding="bgr8")

    def imagePreprocessing(self,img):
        scale = 40
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        width = int(gray.shape[0]*scale/100)
        height = int(gray.shape[1]*scale/100)

        gray = cv2.resize(gray,(height,width))

        gray = cv2.rotate(gray,cv2.ROTATE_180)

        gray =cv2.GaussianBlur(gray,(11,11),0)
        gray = cv2.erode(src=gray,kernel=(9,9) ,iterations=1)
        gray = cv2.dilate(src=gray,kernel=(7,7) ,iterations=1)
        return gray

    def sliceImage(self,img):
        return img[int(self.imgHeight*0.6):,:]

    def sumVertically(self,img):

        sum = img.sum(axis=0)
        sum = np.float32(sum)
        return sum

    def edgeDetection(self,img):


        retval, binary = cv2.threshold(img, 10, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        erotion = cv2.erode(src=binary,kernel=self.medianFilter ,iterations=1)
        dilation = cv2.dilate(src=binary,kernel=self.medianFilter ,iterations=1)
        binarized = self.createImageMask(img,0,retval)
        
        return binarized
        
    def createImageMask(self,image,lowerBound,upperBound):
        return  cv2.inRange(image, lowerBound, upperBound)

    # filter and array to eliminate noise
    def filterWithThreshold(self,gradient,scaleThreshold = 0.4):
        min = np.min(gradient) * scaleThreshold
        max = np.max(gradient) * scaleThreshold
        positive = gradient.copy()
        negative = gradient.copy()
        positive[positive <  max] = 0
        negative[negative > min] = 0

        return positive + negative

    # stay with the positive part of the edge detection
    def filterNegative(self,gradient):
        gradient[gradient < 0 ] = 0
        return gradient
        

    # left shift and compare the arrays
    def shiftCompare(self,gradient):
        shifted = np.roll(gradient.copy(),1) # left shify
        #shifted = np.left_shift(1,gradient) # left shify
        compare = shifted > gradient
        return compare

    def splitEdges(self,gradient,maxScaleFactor = 0.2,minScaleFactor = 0.2):

        maxVal = np.max(gradient)
        minVal = np.min(gradient)


        left_gradient = gradient.copy()
        right_gradient = gradient.copy()


        LEFT_THRESHOLD = -150 if minVal < -150 else -50
        RIGHT_THRESHOLD =  150 if maxVal > 150 else 50

        left_gradient[left_gradient > LEFT_THRESHOLD] = 0
        right_gradient[right_gradient < RIGHT_THRESHOLD] = 0
        


        return left_gradient, right_gradient

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
            #binarized = self.edgeDetection(preprocessedImage)
            #rospy.loginfo(preprocessedImage.shape)
            proprocessedOutput = self.bridge.cv2_to_imgmsg(preprocessedImage)

            
            arrayMessage = Int32MultiArray()
            arrayMessage.data = vertSum
            
            self.preprocessedImagePub.publish(proprocessedOutput)
            self.verticalSumPub.publish(arrayMessage)
            
            
            #rospy.loginfo(vertSum.shape)

            # compute gradient
            gradient = np.gradient(vertSum)

            # split the left edges and the right edges
            left,right = self.splitEdges(gradient)

            #left = self.filterWithNumber(left,up=False)
            #right = self.filterWithNumber(right)

            # compute second gradient
            secondLeftGradient = np.gradient(left)
            secondRightGradient = np.gradient(right)

            # filter noise from second gradient
            secondLeftGradient = self.filterWithThreshold(secondLeftGradient)
            secondRightGradient = self.filterWithThreshold(secondRightGradient)

            # mutiply first with second gradient
            leftMul = left * secondLeftGradient
            rightMul = right * secondRightGradient

            # remove negative portion of both arrays
            leftPositive = self.filterNegative(leftMul)
            rightPositive = self.filterNegative(rightMul)

            # pixelShift:
            leftCompare = self.shiftCompare(leftPositive)
            rightCompare = self.shiftCompare(rightPositive)

            # right and left slices positions
            leftEdges = np.where(leftCompare)
            rightEdges = np.where(rightCompare)

            # Publish left and right edges
            rospy.loginfo("Left: ")
            rospy.loginfo(leftEdges[0])
            rospy.loginfo("Right: ")
            rospy.loginfo(rightEdges[0])
            leftMessage = Float32MultiArray()
            leftMessage.data = leftEdges[0]
            rightMessage = Float32MultiArray()
            rightMessage.data = rightEdges[0]
            self.leftEdgePublisher.publish(leftMessage)
            self.rightEdgePublisher.publish(rightMessage)

if __name__ == '__main__':
    lineDetector = LineDetector()
    try:
        lineDetector.run()
    except rospy.ROSInterruptException:
        pass
