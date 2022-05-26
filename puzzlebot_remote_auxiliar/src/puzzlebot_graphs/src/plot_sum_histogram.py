#!/usr/bin/env python
import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Int32MultiArray



# topico
TOPIC = "/vertical_sum"
# rate
RATE = 10


class Puzzlebot_Grapher:
    def __init__(self):
        #self.vetSumSubscriber = rospy.Subscriber(TOPIC,numpy_msg(Floats),self.veticalSumCallback)
        self.vetSumSubscriber = rospy.Subscriber(TOPIC,Int32MultiArray,self.veticalSumCallback)
        rospy.init_node('puzzlebot_line_detection')

        # Define the ROS node execution rate
        self.rate = rospy.Rate(RATE)
        self.array = None
       
    def veticalSumCallback(self,msg):
        self.array = msg.data

    def run(self):
        plt.figure()
        while not rospy.is_shutdown():
            if self.array is None:
                self.rate.sleep()
            #= np.uint64(self.array)
            gradient = np.gradient(self.array)
            secondGradient = np.gradient(gradient)
            multiplication = gradient*secondGradient
            plt.subplot(3,1,1)
            plt.plot(gradient)
            plt.subplot(3,1,2)
            plt.plot(secondGradient)
            plt.subplot(3,1,3)
            plt.plot(multiplication)
            plt.pause(0.05)
            plt.clf()


if __name__ == '__main__':
    grapher = Puzzlebot_Grapher()

    try:
        grapher.run()
    except rospy.ROSInterruptException:
        pass