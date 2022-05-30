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
    def splitEdges(self,gradient,maxScaleFactor = 0.2,minScaleFactor = 0.2):
        min = np.min(gradient) * minScaleFactor
        max = np.max(gradient) * maxScaleFactor
        mean = np.mean(gradient)
        leftEdges = gradient.copy()
        rightEdges = gradient.copy()
        leftEdges[leftEdges > max ] = mean
        rightEdges[rightEdges < min ] = mean


        return leftEdges,rightEdges


    def filterWithNumber(self,gradient,threshold = 20,up=True):
        filtered = gradient.copy()
        if up:
            filtered[filtered < threshold] = 0
        else:
            filtered[filtered > -1*threshold] = 0

        return filtered


    # filter and array to eliminate noise
    def filterWithThreshold(self,gradient,scaleThreshold = 0.4):
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

    def cleanSignal(self,edges,thres = 10):
        edgeList = list(edges[0])
        cleanEdges = list()
        edge = edgeList[0]
        cleanEdges.append(edge)
        for i in range(1,len(edgeList)):
            diff = abs(edgeList[i]-edge)
            if diff < thres:
                continue
            else:
                edge = edgeList[i]
                cleanEdges.append(edge)
        return np.array(cleanEdges)
        
    def mapEdges(self,leftEdges,rightEdges, minWidth = 40, maxWidth = 70):
        #rospy.loginfo(len(leftEdges))
        #rospy.loginfo(len(rightEdges))

        if len(leftEdges) > 0 and len(rightEdges) > 0:
            mapp = list()
            for i in range(len(leftEdges)):
                for j in range(len(rightEdges)):
                    diff = abs(leftEdges[i] - rightEdges[j])
                    if diff > minWidth and diff < maxWidth and rightEdges[j] != None:
                        mapp.append((leftEdges[i],rightEdges[j]))
                        rightEdges[j] = -1

            return mapp
        else:
            return None

    def run(self):
        plt.figure()
        while not rospy.is_shutdown():
            if self.array is None:
                self.rate.sleep()

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

            # clean from repeated and proximal edges
            leftEdges = self.cleanSignal(leftEdges)
            rightEdges = self.cleanSignal(rightEdges)

            rospy.loginfo("Left Edges:")
            rospy.loginfo(leftEdges)
            rospy.loginfo("Right Edges:")
            rospy.loginfo(rightEdges)



            plt.subplot(2,2,1)
            plt.plot(gradient)
            plt.subplot(2,2,2)
            plt.plot(left)
            plt.plot(right)
            plt.legend(["left","right"])   
            plt.subplot(2,2,3)
            plt.plot(secondLeftGradient)
            plt.plot(secondRightGradient)
            plt.legend(["left","right"])   
            plt.subplot(2,2,4)
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