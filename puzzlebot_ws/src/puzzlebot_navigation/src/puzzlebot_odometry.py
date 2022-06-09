#!/usr/bin/env python

from cv2 import threshold
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
from math import cos, sin, pi

RATE   =  10

RADIUS =  0.05
BASE   =  0.18

class Odometry:
    def __init__(self):
        # Initialize wheel velocity variables
        self.wr = 0.0
        self.wl = 0.0


        ##########################################################################################################
        # TODO: Setup ROS subscribers and publishers, use the callback functions defined bellow if required. 
        #       Your node must publish a Pose2D message to a /pose2d named topic, with the robot's current estimated position info.
        ##########################################################################################################

        # Your code here...
        self.pose_pub = rospy.Publisher("/pose2d",Pose2D,queue_size=1)
        self.wlRecb = rospy.Subscriber("/wl",Float32,self.wl_callback)
        self.wrRecb = rospy.Subscriber("/wr",Float32,self.wr_callback)        

        ##########################################################################################################

        rospy.init_node("puzzlebot_odometry")

        self.pub_rate = 0

        if rospy.has_param('/puzzlebot_controller/parameters/odometry_pub_rate'):
            self.pub_rate = rospy.get_param('/puzzlebot_controller/parameters/odometry_pub_rate')
            rospy.loginfo("Rate value loaded from parameter server, value = %s", self.pub_rate)
        else:
            self.pub_rate = RATE

        self.rate = rospy.Rate(self.pub_rate)

    # Main function
    def run(self):
        # Variable initializations
        position = Pose2D()
        position.theta = 0.0
        position.x = 0.0
        position.y = 0.0

        radius = 0.0
        base = 0.0

        if rospy.has_param('/puzzlebot_controller/parameters/radius'):
            radius = rospy.get_param('/puzzlebot_controller/parameters/radius')
            rospy.loginfo("Radius value loaded from parameter server, value = %s", radius)
        
        else:
            radius = RADIUS

        if rospy.has_param('/puzzlebot_controller/parameters/base'):
            base = rospy.get_param('/puzzlebot_controller/parameters/base')
            rospy.loginfo("Base value loaded from parameter server, value = %s", base)
        
        else:
            base = BASE

        current_time = rospy.get_time()
        last_time = rospy.get_time()

        # Main Loop
        while not rospy.is_shutdown():

            ##########################################################################################################
            # TODO: Calculate current robot position estimation, use atan2 angle convention for the orientation angle.
            ##########################################################################################################

            # Your code here...
            # update time 


            last_time = current_time
            current_time = rospy.get_time()
            dt = current_time-last_time
            # rospy.loginfo("time differential: %s",dt)
            x = position.x
            y = position.y
            theta = position.theta

            if theta <= -pi:
                theta += 2*pi
            elif theta >= pi:
                theta -= 2*pi


            

            position.x  = x + radius * ((self.wr+self.wl)/(2.0)) * cos(theta) * dt
            position.y = y + radius * ((self.wr+self.wl)/(2.0)) * sin(theta) * dt
            position.theta = theta + radius * ((self.wr-self.wl)/(base)) * dt

            """
            position.x = xNew if xNew - x > threshold else x
            position.y = yNew if yNew - y > threshold else y
            """
            ##########################################################################################################

            # Publish message and sleep
            # rospy.loginfo(position)
            self.pose_pub.publish(position)
            self.rate.sleep()

    # Callbacks for wheel velocities and commands
    def wr_callback(self, msg):
        self.wr = msg.data

    def wl_callback(self, msg):
        self.wl = msg.data

if __name__ == "__main__":
    odometry = Odometry()
    try:
        odometry.run()
    except rospy.ROSInterruptException:
        pass