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


    # split the vertical sum into two different arrays based on right and left edges
    def splitEdges(self,gradient,scaleFactor = 0.2):
        min = np.min(gradient) * scaleFactor
        max = np.max(gradient) * scaleFactor
        mean = np.mean(gradient)
        #rospy.loginfo(mean)
        leftEdges = gradient.copy()
        rightEdges = gradient.copy()
        leftEdges[leftEdges > max ] = mean
        rightEdges[rightEdges < min ] = mean


        return leftEdges,rightEdges


    # filter and array to eliminate noise
    def filterWithThreshold(self,gradient,scaleThreshold = 0.3):
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

    def run(self):
        plt.figure()
        while not rospy.is_shutdown():
            if self.array is None:
                self.rate.sleep()
            #= np.uint64(self.array)
            # compute gradient
            gradient = np.gradient(self.array)
            # split the left edges and the right edges
            left,right = self.splitEdges(gradient)
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

            rospy.loginfo(leftEdges)
            rospy.loginfo(rightEdges)

            plt.subplot(3,1,1)
            plt.plot(left)
            plt.plot(right)
            plt.legend(["left","right"])   
            plt.subplot(3,1,2)
            plt.plot(secondLeftGradient)
            plt.plot(secondRightGradient)
            plt.legend(["left","right"])   
            plt.subplot(3,1,3)
            plt.plot(leftPositive)
            plt.plot(rightPositive)
            plt.legend(["left","right"])            
            plt.pause(0.05)
            plt.clf()


if __name__ == '__main__':
    grapher = Puzzlebot_Grapher()

    try:
        grapher.run()
    except rospy.ROSInterruptException:
        pass