#!/usr/bin/env python
import cv2
import numpy as np
import rospy, cv_bridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

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
        self.preprocessedImagePub = rospy.Publisher(OUTPUT_PREPROCESSED_IMAGE_TOPIC,Image,queue_size=10)
        self.edgesImage = rospy.Publisher(OUTPUT_IMAGE_TOPIC,Image,queue_size=10)
        self.lineCheckpoint = rospy.Publisher(OUTPUT_CHECKPOINT_TOPIC,Image,queue_size=10)


        # subscribers 
        self.rawVideoSubscriber = rospy.Subscriber(camera_topic,Image,self.imageCallback)

        # Initialize a rospy node with the name 'puzzlebot_traffic_lights'.
        rospy.init_node('puzzlebot_line_detection')

        # Define the ROS node execution rate
        self.rate = rospy.Rate(pub_rate)

    def imageCallback(self,msg):
        self.image = self.bridge.imgmsg_to_cv2(msg,desired_encoding="bgr8")

    def imagePreprocessing(self,img):
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        gray = cv2.resize(gray,(self.imgWidth,self.imgHeight))
        gray =cv2.GaussianBlur(gray,(7,7),0)

        return gray

    def run(self):
        while not rospy.is_shutdown():
            if self.image is None:
                self.rate.sleep()

            frame = self.image
            preprocessedImage = self.imagePreprocessing(frame)

            proprocessedOutput = self.bridge.cv2_to_imgmsg(preprocessedImage)

            self.preprocessedImagePub.publish(proprocessedOutput)



if __name__ == '__main__':
    lineDetector = LineDetector()
    try:
        lineDetector.run()
    except rospy.ROSInterruptException:
        pass