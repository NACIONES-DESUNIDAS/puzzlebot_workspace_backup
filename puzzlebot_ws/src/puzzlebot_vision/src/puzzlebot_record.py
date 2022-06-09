#!/usr/bin/env python

import cv2
import rospkg
import rospy, cv_bridge
from sensor_msgs.msg import Image

rospack = rospkg.RosPack()

RATE   =  20
CAMERA_TOPIC = '/video_source/raw'

PACKAGE_LOCATION = rospack.get_path('puzzlebot_vision')

class Recorder():
    def __init__(self):
        self.image = None
        self.bridge = cv_bridge.CvBridge()

        self.out = None

        rospy.Subscriber(CAMERA_TOPIC, Image, self.image_callback)

        rospy.init_node('puzzlebot_record')
        rospy.on_shutdown(self.on_shutdown_callback)
        self.rate = rospy.Rate(RATE)

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')	

    def on_shutdown_callback(self):
        self.out.release()
        rospy.loginfo("END")

    def run(self):
        init = True
        while not rospy.is_shutdown():
            if self.image is None: 
                self.rate.sleep()
                continue

            src_frame = self.image

            frame_height = src_frame.shape[0]
            frame_width = src_frame.shape[1]

            if init:
                init = False
                self.out = cv2.VideoWriter(PACKAGE_LOCATION + '/record/puzzlebot_record.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 10, (frame_width,frame_height))
                rospy.loginfo("START")
            self.out.write(src_frame)

            self.rate.sleep()

if __name__ == '__main__':
    recorder = Recorder()
    recorder.run()