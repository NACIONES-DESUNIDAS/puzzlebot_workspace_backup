#!/usr/bin/env python
import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
import numpy as np
import matplotlib.pyplot as plt

# topico
TOPIC = "/vertical_sum"
# rate
RATE = 10


class Puzzlebot_Grapher:
    def __init__(self):
        self.vetSumSubscriber = rospy.Subscriber(TOPIC,numpy_msg(Floats),self.veticalSumCallback)
        rospy.init_node('puzzlebot_line_detection')

        # Define the ROS node execution rate
        self.rate = rospy.Rate(RATE)
        self.array = None
       
    def veticalSumCallback(self,msg):
        self.array = msg.data

    def run(self):
        while not rospy.is_shutdown():
            if self.array is None:
                self.rate.sleep()
            rospy.loginfo(self.array.shape)


if __name__ == '__main__':
    grapher = Puzzlebot_Grapher()

    try:
        grapher.run()
    except rospy.ROSInterruptException:
        pass