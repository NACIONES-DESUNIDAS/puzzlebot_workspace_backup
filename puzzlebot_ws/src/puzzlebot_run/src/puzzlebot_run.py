#!/usr/bin/env python
from distutils import cmd
import rospy
import actionlib
from math import pi
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Pose2D, Twist
from puzzlebot_msgs.msg import GoToPoseAction, GoToPoseGoal, GoToPoseFeedback, GoToPoseResult

ROS_RED_LIGHT_DETECT_TOPIC = '/puzzlebot_vision/traffic_lights/red_light'
ROS_GREEN_LIGHT_DETECT_TOPIC = '/puzzlebot_vision/traffic_lights/green_light'

ROS_SIGNAL_LABEL_TOPIC = '/puzzlebot_vision/traffic_signals/prediction'
ROS_SIGNAL_DETECT_TOPIC = '/puzzlebot_vision/traffic_signals/signal_found'

ROS_INTERSECTION_TOPIC = '/puzzlebot_vision/intersection'

CMD_VEL = '/cmd_vel'
CMD_VEL_GO_2_GOAL = '/cmd_vel/go_2_goal'
CMD_VEL_LINE_DETECT = '/cmd_vel/line_detect'

RATE = 10

class Controller():
    def __init__(self):
        self.pose2d = Pose2D()
        self.pose2d.x = 0.0
        self.pose2d.y = 0.0
        self.pose2d.theta = 0.0

        rospy.init_node('puzzlebot_run')
        self.rate = rospy.Rate(RATE)

        ##################

        self.client = actionlib.SimpleActionClient('go2pose', GoToPoseAction)

        ##################

        self.client.wait_for_server()

        self.dist = 0.6

        self.action = False
        
        self.STATE = 0

        ##################

        self.poseSub = rospy.Subscriber("/pose2d",Pose2D,self.poseCallback)

        ##################

        self.cmd_vel_go_2_goal = rospy.Subscriber(CMD_VEL_GO_2_GOAL,Twist,self.go_2_goal_callback)
        self.cmd_vel_line_detect = rospy.Subscriber(CMD_VEL_LINE_DETECT,Twist,self.line_detect_callback)

        self.go_2_goal = Twist()
        self.line_detect = Twist()

        ##################

        self.redLightFlag = rospy.Subscriber(ROS_RED_LIGHT_DETECT_TOPIC, Bool, self.redLightFlag_callback)
        self.greenLightFlag = rospy.Subscriber(ROS_GREEN_LIGHT_DETECT_TOPIC, Bool, self.greenLightFlag_callback)

        self.redFlag = Bool()
        self.greenFlag = Bool()

        ##################

        self.signalFlag = rospy.Subscriber(ROS_SIGNAL_DETECT_TOPIC, Bool, self.signalFlag_callback)
        self.signalLabel = rospy.Subscriber(ROS_SIGNAL_LABEL_TOPIC, String, self.signalLabel_callback)

        self.sigFlag = Bool()
        self.sigLabel = String

        ##################

        self.intersectionFlag = rospy.Subscriber(ROS_INTERSECTION_TOPIC, Bool, self.intersection_callback)

        self.interFlag = Bool()

        ##################

        self.cmd_vel_pub = rospy.Publisher(CMD_VEL, Twist, queue_size=1)
    
    def run(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.0
        cmd_vel.linear.z = 0.0
        cmd_vel.angular.x = 0.0
        cmd_vel.angular.y = 0.0
        cmd_vel.angular.z = 0.0

        ##################

        goal_pose = Pose2D()
        goal_pose.x = 0.0
        goal_pose.y = 0.0
        goal_pose.theta = 0.0
        curr_pose = Pose2D()
        curr_pose.x = self.pose2d.x
        curr_pose.y = self.pose2d.y
        curr_pose.theta = self.pose2d.theta

        ##################

        cmd_vel.linear.x = self.line_detect.linear.x
        cmd_vel.angular.z = self.line_detect.angular.z

        if self.sigLabel == "aplastame":
            cmd_vel.linear.x *= 1.5
        elif self.sigLabel == "stop_signal":
            cmd_vel.linear.x = 0
            cmd_vel.angular.z = 0

        if not self.action:
            if self.sigLabel == "up_signal":
                self.STATE = 0
            elif self.sigLabel == "right_signal":
                self.STATE = 1

        success = False
        if self.action and not success and not self.redFlag:
            if self.STATE == 0:
                goal_pose.x = curr_pose.x + self.dist
                goal_pose.y = curr_pose.y
                goal_pose.theta = curr_pose.theta
            elif self.STATE == 1:
                goal_pose.x = curr_pose.x
                goal_pose.y = curr_pose.y - (self.dist / 2.0) - 0.2
                goal_pose.theta = curr_pose.theta - (pi / 2.0)
            else:
                goal_pose.x = curr_pose.x - (self.dist / 2.0)
                goal_pose.y = curr_pose.y
                goal_pose.theta = curr_pose.theta

            goal = GoToPoseGoal(goal_pose2d = goal_pose)
            self.client.send_goal(goal = goal, feedback_cb = self.callback_feedback, active_cb = self.callback_active)
            self.client.wait_for_result()
            result = self.client.get_result()

            if result.success:
                if self.STATE == 0:
                    success = True
                    self.action = False
                    self.STATE += 1
                elif self.STATE == 1:
                    self.STATE += 1
                else:
                    success = True
                    self.action = False
                    self.STATE = 0

        # rospy.loginfo("z: %s", cmd_vel.angular.z)

        self.cmd_vel_pub.publish(cmd_vel)
        self.rate.sleep()

    def go_2_goal_callback(self, msg):
        self.go_2_goal = msg
    
    def line_detect_callback(self, msg):
        self.line_detect = msg

    def redLightFlag_callback(self, msg):
        self.redFlag = msg.data
        # rospy.loginfo("Red: %s",self.redFlag)

    def greenLightFlag_callback(self, msg):
        self.greenFlag = msg.data
        # rospy.loginfo("Green: %s",self.greenFlag)

    def signalFlag_callback(self, msg):
        self.sigFlag = msg.data
        # rospy.loginfo("Signal: %s",self.sigFlag)

    def signalLabel_callback(self, msg):
        self.sigLabel = msg.data
        rospy.loginfo("Label: %s", self.sigLabel)

    def intersection_callback(self, msg):
        self.interFlag = msg.data
        if self.interFlag:
            self.action = True
        # rospy.loginfo("Intersection: %s", self.interFlag)

    def poseCallback(self, msg):
        self.pose2d = msg

    def callback_active(self):
        rospy.loginfo("Action server is processing the goal")

    def callback_feedback(self, feedback):
        rospy.loginfo("Feedback:%s" % str(feedback))

if __name__ == '__main__':
    try:
        controller = Controller()
        while True:
            controller.run()
    except rospy.ROSInterruptException:
        pass

