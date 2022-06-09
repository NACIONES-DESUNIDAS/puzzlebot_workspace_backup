#!/usr/bin/env python3


import rospy, cv2

from sensor_msgs import Image
from std_msgs import String
import numpy as np
from tensorflow.keras.models import load_model
import tensorflow as tf
import os
#from sign_rec.srv import check, data


# Constant definition
frameWidth = 640  # CAMERA RESOLUTION
frameHeight = 480
brightness = 180

#device = tf.config.list_physical_devices('GPU')
#tf.config.experimental.set_memory_growth(device[0], True)
#tf.config.experimental.set_virtual_device_configuration(device[0], [tf.config.experimental.VirtualDeviceConfiguration(memory_limit=1024)])

model = load_model("/home/puzzlebot/puzzlebot_ws/src/puzzlebot_vision/src/puzzlebot_model.h5")


ROS_LABEL_PUBLISHER = "/puzzlebot/traffic_signals/predictions/"


ROS_WHITE_SIGNAL_IMAGE = "/puzzlebot_vision/traffic_signals/white_image"
ROS_RED_SIGNAL_IMAGE = "/puzzlebot_vision/traffic_signals/red_image"
ROS_BLUE_SIGNAL_IMAGE = "/puzzlebot_vision/traffic_signals/blue_image"


ROS_SIGNAL_DETECT_TOPIC = '/puzzlebot_vision/traffic_signals/red_signal_found'
ROS_BLUE_SIGNAL_DETECT_TOPIC = '/puzzlebot_vision/traffic_signals/blue_signal_found'
ROS_WHITE_SIGNAL_DETECT_TOPIC = '/puzzlebot_vision/traffic_signals/white_signal_found'


class DetectStop():

    def format_image(self,data):
        aux_img = self.imgmsg_to_cv2(data)
        aux_img = cv2.resize(aux_img, (frameWidth, frameHeight))
        return aux_img

    def imgmsg_to_cv2(self,msg):
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)

    def grayscale(self,img):
        # Grayscale the image

        return img

    def equalize(self,img):
        # Equalize the histogram

        return img

    def preprocessing(self,img):
        # TODO: Write grayscale and equalize functions using cv2
        img = cv2.resize(img, (32, 32))
        img = self.grayscale(img)
        img = self.equalize(img)
        img = img / 255
        # the image needs to be reshaped to be used as an input tensor for tensorflow
        return img.reshape(1, 32, 32, 1)

    def __init__(self):

        self.labelsPub = rospy.Publisher(ROS_LABEL_PUBLISHER,String,queue_size=10)

        """
        self.redImgSub = rospy.Subscriber(ROS_RED_SIGNAL_IMAGE,Image)
        self.blueImgSub = rospy.Subscriber(ROS_BLUE_SIGNAL_IMAGE,Image)
        self.whiteImgSub = rospy.Subscriber(ROS_WHITE_SIGNAL_IMAGE,Image)


        self.redFlagSub = rospy.Subscriber(ROS_RED_SIGNAL_IMAGE,String)
        self.blueFlagSub = rospy.Subscriber(ROS_BLUE_SIGNAL_IMAGE,String)
        self.whiteFlagSub = rospy.Subscriber(ROS_WHITE_SIGNAL_IMAGE,String)
        """
        redFlag = None
        blueFlag = None
        greenFlag = None

        redImage = None
        blueImage = None
        whiteImage = None

    

    # function callbacks 

    def run(self):
        rospy.loginfo("run")


if __name__ == '__main__':
    predictor = DetectStop()
    try:
        predictor.run()
    except rospy.ROSInterruptException:
        pass
