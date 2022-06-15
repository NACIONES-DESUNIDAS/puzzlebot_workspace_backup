#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
from cv2 import cvtColor
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




# ROS Topics used to publish the desired output messages

"""
Tópicos de salida:
Imagen con circulos
Imagen segementada
Señal detectada
"""
ROS_IMAGE_OUTPUT_TOPIC = '/puzzlebot_vision/traffic_signals/image_segmentation'
ROS_IMAGE_SQUARE_TOPIC = "/puzzlebot_vision/traffic_signals/bounding_boxes"





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

        self.outputImagePub = rospy.Publisher(ROS_IMAGE_OUTPUT_TOPIC,Image,queue_size=10)
        self.boxesImagePub = rospy.Publisher(ROS_IMAGE_SQUARE_TOPIC,Image,queue_size=10)


        # Initialize a rospy node with the name 'puzzlebot_traffic_lights'.
        rospy.init_node('puzzlebot_traffic_signals')

        # Define the ROS node execution rate
        self.rate = rospy.Rate(pub_rate)




    # COnverts to grayscale, scales the image to reduce processing time and rotates
    def preprocessImage(self,img):

        width = int(img.shape[0]*self.img_scale_factor/100)
        height = int(img.shape[1]*self.img_scale_factor/100)
        img = cv2.resize(img,(height,width))
        img = cv2.rotate(img,cv2.ROTATE_180)

        return img


    # receibes image and gets a list of potential traffic signals
    def getHoughCircles(self,img):
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        #gray = cv2.GaussianBlur(gray,(7,7),0)
        circles_img = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,1,200,param1=50,param2=30,minRadius=25,maxRadius=30)
        #circles_img = np.uint16(np.around(circles_img))
        return circles_img




    # GETS THE IMAGE SEGEMENTATION AND PUBLISHES IT
    def pubSegementedImages(self,circles,img):
        images = list()
        rectangles = list()
        outputImg = img.copy()
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for circle in circles:
                x = circle[0]
                y = circle[1]
                rad = circle[2] + 10
                topLeftCorner = (x-(rad),y-(rad))
                bottomRightCorner =  (x+(rad),y+(rad))
                outputImg = cv2.rectangle(outputImg,topLeftCorner,bottomRightCorner,(255,0,0),5)
                segemented = img.copy()[y-rad:y+rad,x-rad:x+rad,:]
                
                try:
                    segmentedOutput = self.bridge.cv2_to_imgmsg(segemented,encoding="bgr8")
                    self.outputImagePub.publish(segmentedOutput)
                except:
                    continue           
                
            output = self.bridge.cv2_to_imgmsg(outputImg,encoding="bgr8")
            self.boxesImagePub.publish(output)








    def run(self):
        
        while not rospy.is_shutdown():
            if self.image is None: 
                self.rate.sleep()
                continue
        
            frame = self.image

            img = self.preprocessImage(frame)

            circles = self.getHoughCircles(img)


            self.pubSegementedImages(circles,img)
            #self.rate.sleep()


    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg,desired_encoding="bgr8")


if __name__ == '__main__':
    traffic_signal_detector = Signal_Identifier()
    
    try:
        traffic_signal_detector.run()
    except rospy.ROSInterruptException
