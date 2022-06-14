#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray, Float32

FRAMEWIDTH = 512
EDGES_THRESHOLD_MIN = 17
EDGES_THRESHOLD_MAX = 45
EDGE_DISTANCE_THRESHOLD = 140

NAN = np.nan

PUB_RATE = 30

class Modulator():
    def __init__(self):
        # Subscribers
        self.leftEdgeSubscriber = rospy.Subscriber("/leftEdge", Float32MultiArray, self.leftEdgeCallback)
        self.rightEdgeSubscriber = rospy.Subscriber("/rightEdge", Float32MultiArray, self.rightEdgeCallback)

        # Publishers
        self.angularErrorPub = rospy.Publisher("/angularError", Float32, queue_size = 10)

        self.rightEdges = None
        self.leftEdges = None

        self.currentLinePos = 0
        self.previousLinePos = 0

        self.angularErrorRecord = 0

        rospy.init_node('edgeModulator')
        self.rate = rospy.Rate(PUB_RATE)

    def rightEdgeCallback(self, msg):
        self.rightEdges = msg.data

    def leftEdgeCallback(self, msg):
        self.leftEdges = msg.data

    def edgeModulation(self):
        line_edges_tuple = []
        filtered_line_edges_tuple = {
            "left" : {
                "line_edges" : None,
                "line_position" : None
            }, "center" : {
                "line_edges" : None,
                "line_position" : None
            }, "right" : {
                "line_edges" : None,
                "line_position" : None
            }
        }
        sort_func = lambda x : x[1]
        for left_edge_point in self.leftEdges:
            line_edges_tuple.append(("LEFT", left_edge_point))
        for right_edge_point in self.rightEdges:
            line_edges_tuple.append(("RIGHT", right_edge_point))
        line_edges_tuple.sort(key = sort_func)

        # rospy.loginfo(line_edges_tuple)
        
        for i in range(0, len(line_edges_tuple) - 1):
            # Current and next edge
            curr_edge = line_edges_tuple[i]
            next_edge = line_edges_tuple[i+1]
            # Edges types
            curr_edge_type = curr_edge[0]
            next_edge_type = next_edge[0]
            # Edges values
            curr_edge_value = curr_edge[1]
            next_edge_value = next_edge[1]
            # Edge difference
            line_diff = next_edge_value - curr_edge_value
            # Check for alternating edges
            if curr_edge_type == "LEFT" and next_edge_type == "RIGHT" and curr_edge_value > 100 and curr_edge_value < 412:
                # rospy.loginfo((curr_edge, next_edge))
                if line_diff > EDGES_THRESHOLD_MIN and line_diff < EDGES_THRESHOLD_MAX:
                    # rospy.loginfo((curr_edge, next_edge))
                    filtered_line_edges_tuple["center"]["line_edges"] = (curr_edge, next_edge)
                    filtered_line_edges_tuple["center"]["line_position"] = ((next_edge_value + curr_edge_value) / 2.0) - (FRAMEWIDTH / 2.0)
            elif curr_edge_type == "RIGHT" and next_edge_type == "LEFT":
                if line_diff > EDGE_DISTANCE_THRESHOLD:
                    if line_diff < FRAMEWIDTH / 2:
                        filtered_line_edges_tuple["left"]["line_edges"] = (None, curr_edge)
                        filtered_line_edges_tuple["left"]["line_position"] = None
                    else:
                        filtered_line_edges_tuple["left"]["line_edges"] = (next_edge, None)
                        filtered_line_edges_tuple["left"]["line_position"] = None
        
        # rospy.loginfo(filtered_line_edges_tuple)

        if filtered_line_edges_tuple["center"]["line_position"] == None:
            self.currentLinePos == self.previousLinePos
            self.angularError = np.nan
        else:
            self.currentLinePos = filtered_line_edges_tuple["center"]["line_position"]
            self.angularError = filtered_line_edges_tuple["center"]["line_position"]

        self.previousLinePos = self.currentLinePos

        self.angularErrorRecord = self.angularError

        # rospy.loginfo(self.angularError)

        self.angularErrorPub.publish(self.angularError)

    def run(self):
        while not rospy.is_shutdown():
            if self.rightEdges is None or self.leftEdges is None:
                self.rate.sleep()
                continue
            
            self.edgeModulation()
            self.rate.sleep()

if __name__ == '__main__':
    modulator = Modulator()
    try:
        modulator.run()
    except rospy.ROSInterruptException:
        pass