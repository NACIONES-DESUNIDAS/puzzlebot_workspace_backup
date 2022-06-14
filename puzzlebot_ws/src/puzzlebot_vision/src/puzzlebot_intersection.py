#!/usr/bin/env python
import cv2
import numpy as np
import rospy, cv_bridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool



# camera topic
CAMERA_TOPIC = '/video_source/raw'
INTERSECTION_OUTPUT_FLAG_TOPIC= "/puzzlebot_vision/intersection"
INTERSECTION_OUTPUT_IMAGE_TOPIC = "/puzzlebot_vision/intersection/draw_contours"


# usefull marameters
RATE   =  30
THRESHOLD = 3
MIN_Y = 35
MAX_Y = 65
MIN_AREA = 250
MAX_AREA = 600
IMG_SCALE_FACTOR = 50


class IntersectionDetector():
    def __init__(self):

        # video sibcriber
        self.rawVideoSubscriber = rospy.Subscriber(CAMERA_TOPIC,Image,self.imageCallback)

        # signal publisher
        self.detectedFlagPub = rospy.Publisher(INTERSECTION_OUTPUT_FLAG_TOPIC,Bool,queue_size=1)
        # image debug
        self.imageDebugPub = rospy.Publisher(INTERSECTION_OUTPUT_IMAGE_TOPIC,Image,queue_size= 1)

        # bridge 
        self.bridge = cv_bridge.CvBridge()
        self.image = None

        # resize 
        self.scale = IMG_SCALE_FACTOR

        # filtering parameters
        self.threshold = THRESHOLD

        self.miny = MIN_Y
        self.maxY = MAX_Y

        self.minArea = MIN_AREA
        self.maxArea = MAX_AREA

        rospy.init_node("puzzlebot_intersection_detector")
        self.rate = rospy.Rate(RATE)



    def imageCallback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')	


    def imagePreprocessing(self,img):
        scale = 50
        width = int(img.shape[0]*self.scale/100)
        height = int(img.shape[1]*self.scale/100)
        img = cv2.resize(img,(height,width))
        rot = cv2.rotate(img,cv2.ROTATE_180)

        gray = cv2.cvtColor(rot,cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)
        gray =cv2.GaussianBlur(gray,(11,11),0)
        return gray,rot


    def sliceImage(self,img):
        h = img.shape[0]
        return img[int(h*0.4):,:]




    def thresholdImg(self,img):
        retval, binary = cv2.threshold(img, 50, 255, cv2.THRESH_BINARY)
        return binary


    def edgeDetection(self,img):


        filterC = list()
        minY = self.miny
        maxY = self.maxY
        minArea = self.minArea
        maxArea = self.maxArea


        contours,hierchy = cv2.findContours(img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)  



        #contours = [contour for contour in contours if cv2.contourArea(contour) < 600 and cv2.contourArea(contour) > 250] 
        #  (216, 640, 3)
        for contour in contours:

            try:
                M = cv2.moments(contour)
                y = int(M["m01"]/M["m00"])
            except:
                continue


            app = False
            app2 = False


            area = cv2.contourArea(contour)
            if area < maxArea and area > minArea:
                app = True
            
            
            if y > minY and y < maxY:
                app2 = True

            if app and app2:
                filterC.append(contour)

        
        return filterC
            

    def run(self):
        while not rospy.is_shutdown():
  
            if self.image is None:
                self.rate.sleep()
                continue
            
            frame = self.image

            # escalar y rotar a 180 grados
            gray,frameP = self.imagePreprocessing(frame)

            # area de interes
            slicedGray = self.sliceImage(gray)
            oriSliced = self.sliceImage(frameP)

            # aplicar threshold 
            threshold = self.thresholdImg(slicedGray)
            # obtnener contornos
            edges = self.edgeDetection(threshold)
            
            contourImage = cv2.drawContours(oriSliced,edges,-1,(0,0,255),2)

            flag = True if len(edges) > self.threshold else False

            self.detectedFlagPub.publish(flag)

            debug = self.bridge.cv2_to_imgmsg(contourImage,encoding="bgr8")
            self.imageDebugPub.publish(debug)


if __name__ == '__main__':
    intersectionDetector = IntersectionDetector()
    
    try:
        intersectionDetector.run()
    except rospy.ROSInterruptException:
        pass
    
