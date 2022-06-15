#!/usr/bin/env python3

import queue
import rospy
import cv2

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

model = load_model("/home/lftronic/Documents/code/ROS/puzzlebot_workspace_backup/puzzlebot_ws/src/puzzlebot_vision/src/puzzlebot_model.h5")

# Subscriber
ROS_IMAGE_OUTPUT_TOPIC = '/puzzlebot_vision/traffic_signals/image_segmentation'
# Ros Publisher
ROS_SIGNAL_DETECT_TOPIC = '/puzzlebot_vision/traffic_signals/signal_found'
ROS_LABEL_PUBLISHER = "/puzzlebot_vision/traffic_signals/prediction"

# parameter server stuff (quizas algun dia...)
IMG_SHAPE = (150,150,1)
IMG_TUPPLE_SHAPE = (150,150)
dims = 150
RATE = 30

class DetectStop():

    def __init__(self):

        self.image = None
        self.flag = None

        self.imgSub = rospy.Subscriber(ROS_IMAGE_OUTPUT_TOPIC,Image,self.imageCallback)

        self.labelPub = rospy.Publisher(ROS_LABEL_PUBLISHER,String,queue_size=1)
        self.detectedFlagPub = rospy.Publisher(ROS_SIGNAL_DETECT_TOPIC,Bool,queue_size=1)

        # classatributes
        self.lastLabel = None
        self.labelCounter = 0
        self.labelThreshold = 5

        rospy.init_node("puzzlebot_predictor_node")
        self.rate = rospy.Rate(RATE)

    def imageCallback(self,img):
        self.image = np.frombuffer(img.data, dtype=np.uint8).reshape(img.height, img.width, -1)

    def imageProcessing(self,image):
        image = image.astype(np.uint8)
        gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)
        gray = cv2.resize(gray,IMG_TUPPLE_SHAPE)
        gray = gray.reshape(1,dims,dims,1)
        return gray.astype(np.float64)/255

    def getCalssName(self, classNo):
        labels = {0: 'stop_signal', 1: 'aplastame', 2: 'right_signal', 3: 'left_signal', 4: 'up_signal', 5: 'around_signal'}
        return labels[classNo]

    def pubLabelFlag(self,label):
        if self.lastLabel == label:
            self.labelCounter += 1
        if self.labelCounter > self.labelThreshold:
            self.labelCounter = 0
            flag = True if self.lastLabel != "not_found" else False
            self.labelPub.publish(self.lastLabel)
            self.detectedFlagPub.publish(flag)

        self.lastLabel = label

    # function callbacks 

    def run(self):

        while not rospy.is_shutdown():
            if self.image is None:
                self.rate.sleep()
                continue

            frame = self.image
            img = self.imageProcessing(frame)

            prediction = model.predict(img)
            index = np.argmax(prediction)
            label = self.getCalssName(index)
            proba = prediction[0][index]
            rospy.loginfo(proba)
            rospy.loginfo(label)
            label = "not_found" if proba > 0.95 else label
            self.labelPub.publish(label)
            self.pubLabelFlag(label=label)
           
            self.image = None
            self.rate.sleep()

if __name__ == '__main__':
    predictor = DetectStop()
    predictor.run()
