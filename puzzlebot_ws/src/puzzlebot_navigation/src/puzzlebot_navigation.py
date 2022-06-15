#!/usr/bin/env python
import rospy
import actionlib
import numpy as np
import math
from math import atan2, pi, sqrt
from geometry_msgs.msg import Pose2D, Twist
from std_msgs.msg import Float32
from puzzlebot_msgs.msg import GoToPoseAction, GoToPoseFeedback, GoToPoseResult

RATE   =  10

linealVel = 0.1
kp = 0.00075
kd = 0.000075

TIME_THRESHOLD = 4.5

THETA_THRESHOLD = 10.0 * pi / 180.0
DIST_THRESHOLD = 0.1

COUNTER_THRESHOLD = 4

CMD_VEL_GO_2_GOAL = "/cmd_vel"
CMD_VEL_LINE_DETECT = "/cmd_vel/line_detect"

class Navigator():
    def __init__(self):
        self.pose2d = Pose2D()
        self.pose2d.x = 0.0
        self.pose2d.y = 0.0
        self.pose2d.theta = 0.0

        self.angularTimeTolerance = 0

        self.pastAngularError = 0
        self.pastAngularErrorAbs = 0

        self.counter = 5
        self.counterNAN = 0

        ##################

        # Your code here...
        self.cmd_vel_go_2_goal_pub = rospy.Publisher(CMD_VEL_GO_2_GOAL,Twist,queue_size=1)
        self.cmd_vel_line_detect_pub = rospy.Publisher(CMD_VEL_LINE_DETECT,Twist,queue_size=1)
        self.poseRecb = rospy.Subscriber("/pose2d",Pose2D,self.poseCallback)

        ##################

        rospy.init_node('puzzlebot_navigation')

        self.currentAngularTime = rospy.get_time()
        self.pastAngularTime = rospy.get_time()

        self.pub_rate = 0

        if rospy.has_param('/puzzlebot_controller/parameters/navigator_pub_rate'):
            self.pub_rate = rospy.get_param('/puzzlebot_controller/parameters/navigator_pub_rate')
            rospy.loginfo("Rate value loaded from parameter server, value = %s", self.pub_rate)
        else:
            self.pub_rate = RATE

        if rospy.has_param('/puzzlebot_navigation/parameters/linearVel'):
            self.linealVel = rospy.get_param('/puzzlebot_navigation/parameters/linearVel')
            rospy.loginfo("Rate value loaded from parameter server, value = %s", self.linealVel)
        else:
            self.linealVel = linealVel

        ##################

        self.feedback = GoToPoseFeedback()
        self.result = GoToPoseResult()
        self.action = actionlib.SimpleActionServer(name = "go2pose",ActionSpec= GoToPoseAction,execute_cb= self.actionCallback,auto_start=False)

        ##################

        self.angularErrorSub = rospy.Subscriber("/angularError", Float32, self.lineNavigationCallback)

        self.action.start()
        self.rate = rospy.Rate(self.pub_rate)
        rospy.spin()

    def poseCallback(self, data):
        self.pose2d = data

    def lineNavigationCallback(self, msg):
        self.pastAngularTime = self.currentAngularTime
        self.currentAngularTime = rospy.get_time()
        angularTempDiff = self.currentAngularTime - self.pastAngularTime
        angularError = msg.data 
        angularErrorAbs = abs(angularError) if not math.isnan(angularError) else self.pastAngularErrorAbs

        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.0
        cmd_vel.linear.z = 0.0
        cmd_vel.angular.x = 0.0
        cmd_vel.angular.y = 0.0
        cmd_vel.angular.z = 0.0

        controlAngularSpeed = kp * angularErrorAbs + (kd * (angularErrorAbs - self.pastAngularErrorAbs)) / angularTempDiff

        if angularError > 0:
            factor = -1.0
        elif angularError < 0:
            factor = 1.0
        else:
            factor = 0

        if not math.isnan(angularError):
            self.counterNAN = 0
            if self.counter >= COUNTER_THRESHOLD and self.counterNAN <= COUNTER_THRESHOLD:
                self.pastAngularErrorAbs = angularErrorAbs
                self.pastAngularError = angularError
            else:
                if self.pastAngularError > 0:
                    factor = -1.0
                elif self.pastAngularError < 0:
                    factor = 1.0
                else:
                    factor = 0
            self.counter += 1
        else:
            self.counter = 0
            if self.pastAngularError > 0:
                factor = -1.0
            elif self.pastAngularError < 0:
                factor = 1.0
            else:
                factor = 0
            self.counterNAN += 1
        
        if math.isnan(angularError):
            if self.angularTimeTolerance < TIME_THRESHOLD:
                self.angularTimeTolerance += angularTempDiff
                cmd_vel.angular.z =  0.075 * factor
                cmd_vel.linear.x = 0.1
            else:
                cmd_vel.linear.x = 0
                cmd_vel.angular.z = 0
        else:
            self.angularTimeTolerance = 0
            cmd_vel.angular.z = factor * controlAngularSpeed if controlAngularSpeed <= 0.1 else 0.1 * factor
            cmd_vel.linear.x = 0.1

        self.cmd_vel_line_detect_pub.publish(cmd_vel)
        
    def actionCallback(self, goal):
        x_goal = goal.goal_pose2d.x
        y_goal = goal.goal_pose2d.y
        theta_goal = goal.goal_pose2d.theta

        self.rate = rospy.Rate(RATE)
        success = False

        STATE = 0
        current_time = rospy.get_time()
        last_time = rospy.get_time()
        theta_err_p = 0
        theta_err2_p = 0
        while not success:
            if self.action.is_preempt_requested():
                self.action.set_preempted()
                success = False
                break

            cmd_vel = Twist()
            cmd_vel.linear.x = 0.0
            cmd_vel.linear.y = 0.0
            cmd_vel.linear.z = 0.0
            cmd_vel.angular.x = 0.0
            cmd_vel.angular.y = 0.0
            cmd_vel.angular.z = 0.0

            x_curr = self.pose2d.x
            y_curr = self.pose2d.y
            theta_curr = self.pose2d.theta

            current_time = rospy.get_time()
            dt = current_time - last_time
            last_time = current_time
            Kp = 0.1
            Kd = 0.05
            theta_obj = atan2(y_goal - y_curr, x_goal - x_curr)
            theta_err = theta_obj - theta_curr
            if theta_err >= pi:
                theta_err -= 2*pi
            elif theta_err <= -pi:
                theta_err += 2*pi
            theta_err2 = theta_goal - theta_curr
            if theta_err2 >= pi:
                theta_err2 -= 2*pi
            elif theta_err2 <= -pi:
                theta_err2 += 2*pi
            dist_err = sqrt((y_goal - y_curr)**2+(x_goal - x_curr)**2)
            if STATE == 0:
                if dt == 0: dt = 0.00001
                cmd_vel.angular.z = (Kp * theta_err) + (Kd * ((theta_err-theta_err_p) / dt))
                cmd_vel.linear.x = 0
                theta_err_p = theta_err
                if abs(theta_err) < THETA_THRESHOLD: 
                    STATE += 1
            elif STATE == 1:
                cmd_vel.angular.z = (Kp * theta_err) + (Kd * ((theta_err-theta_err_p) / dt))
                cmd_vel.linear.x = 0.1
                theta_err_p = theta_err
                if dist_err <= DIST_THRESHOLD:
                    STATE += 1
            elif STATE == 2:
                if dt == 0: dt = 0.00001
                cmd_vel.angular.z = (Kp * theta_err2) + (Kd * ((theta_err2-theta_err2_p) / dt))
                cmd_vel.linear.x = 0
                theta_err2_p = theta_err2 
                if abs(theta_err2) < THETA_THRESHOLD: 
                    STATE += 1
            else:
                cmd_vel.angular.z = 0
                cmd_vel.linear.x = 0
                success = True
                print(success)

            if cmd_vel.angular.z >= 0.1: 
                cmd_vel.angular.z = 0.1 
            elif cmd_vel.angular.z <= -0.1: 
                cmd_vel.angular.z = -0.1 
            self.cmd_vel_go_2_goal_pub.publish(cmd_vel)

            self.feedback.current_pose2d = self.pose2d
            self.action.publish_feedback(self.feedback)
            self.rate.sleep()

        if success:
            self.result.success = True
            self.action.set_succeeded(self.result)

if __name__ == '__main__':
    try:
        navigator = Navigator()
    except rospy.ROSInterruptException:
        pass
