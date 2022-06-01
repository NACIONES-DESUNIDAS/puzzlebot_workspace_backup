#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray, Float32

FRAMEWIDTH = 512
EDGES_THRESHOLD_MIN = 26
EDGES_THRESHOLD_MAX = 28

class Modulator():
    def __init__(self):
        # Subscribers
        self.leftEdgeSubscriber = rospy.Subscriber("/leftEdge", Float32MultiArray, self.leftEdgeCallback)
        self.rightEdgeSubscriber = rospy.Subscriber("/rightEdge", Float32MultiArray, self.rightEdgeCallback)

        # Publishers
        self.angularErrorPub = rospy.Publisher("/angularError", Float32, queue_size = 10)

        self.currentLinePos = 256

        rospy.init_node('edgeModulator')
        rospy.spin()

    def rightEdgeCallback(self, msg):
        self.rightEdges = msg.data

    def leftEdgeCallback(self, msg):
        self.leftEdges = msg.data

    def edgeModulation(self):
        if len(self.rightEdges) == len(self.leftEdges):
            self.centerPoint = self.leftEdges - self.rightEdges
            self.lineWidth = self.leftEdges - self.rightEdges
            self.linePos = ((self.leftEdges - self.rightEdges) / 2.0) - (FRAMEWIDTH / 2.0)
        else:
            self.centerPoint = np.nan()

        self.previousLinePos = self.currentLinePos
        self.currentLinePos = self.linePos
        if (self.previousLinePos > self.currentLinePos):
            self.angularError = self.previousLinePos - self.currentLinePos
        elif (self.previousLinePos < self.currentLinePos):
            self.angularError = self.currentLinePos - self.previousLinePos
            self.angularError *= -1.0
        else:
            self.angularError = 0

if __name__ == '__main__':
    try:
        modulator = Modulator()
    except rospy.ROSInterruptException:
        pass