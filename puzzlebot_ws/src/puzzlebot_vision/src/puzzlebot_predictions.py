#!/usr/bin/env python3


import queue
import rospy, cv2

from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
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


# ROS SUbscribers
ROS_IMAGE_OUTPUT_TOPIC = '/puzzlebot_vision/traffic_signals/image_segmentation'
ROS_SIGNAL_DETECT_TOPIC = '/puzzlebot_vision/traffic_signals/signal_found'



RATE = 1
IMG_SIZE = 150
IMG_LABELS = ["aplastame","around_signal","left_signal","right_signal","stop_signal","up_signal"]


class DetectStop():


    def __init__(self):


        self.image = None
        self.flag = None

        self.imgSub = rospy.Subscriber(ROS_IMAGE_OUTPUT_TOPIC,Image,self.imageCallback)
        self.flagSub = rospy.Subscriber(ROS_SIGNAL_DETECT_TOPIC,Bool,self.flagCallback)

        self.labelPub = rospy.Publisher(ROS_LABEL_PUBLISHER,String,queue_size=1)

        rospy.init_node("puzzlebot_predictor_node")
        self.rate = rospy.Rate(RATE)



    def imageCallback(self,img):
        self.img = np.frombuffer(img.data, dtype=np.uint8).reshape(img.height, img.width, -1)
        rospy.loginfo(self.img.shape)
        
    def flagCallback(self,msg):
        self.flag = msg.data
        rospy.loginfo(msg)
        


    def imgmsg_to_cv2(self,msg):
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)



    def resize(self,img):
        # Equalize the histogram
        img = cv2.resize(self.imgSize,self.imgSize)
        img = cv2.reshape(1,self.imgSize,self.imgSize,3)
        return img


    def preprocessing(self,image):
        image = image.astype(np.uint8)
        image_hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

        H, S, V = cv2.split(image_hsv)

        eq_H = cv2.equalizeHist(H)
        eq_S = cv2.equalizeHist(S)
        eq_V = cv2.equalizeHist(V)

        image_eq_hsv = cv2.merge([H, S, eq_V])
        image_eq_hsv = cv2.cvtColor(image_eq_hsv, cv2.COLOR_HSV2BGR)

        return image_eq_hsv.astype(np.float64)/255


    def imagePredict(self,img):
        return self.labels[np.argmax(model.predict(img))]


    # function callbacks 

    def run(self):
        rospy.loginfo("run")


if __name__ == '__main__':
    predictor = DetectStop()
    try:
        predictor.run()
    except rospy.ROSInterruptException:
        pass