#!/usr/bin/env python
import rospy
import actionlib
from math import pi
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Pose2D, Twist
from puzzlebot_msgs.msg import GoToPoseAction, GoToPoseGoal, GoToPoseFeedback, GoToPoseResult

ROS_RED_LIGHT_DETECT_TOPIC = '/puzzlebot_vision/traffic_lights/red_light'
ROS_GREEN_LIGHT_DETECT_TOPIC = '/puzzlebot_vision/traffic_lights/green_light'

ROS_SIGNAL_DETECT_TOPIC = '/puzzlebot_vision/traffic_signals/signal_found'
ROS_SIGNAL_LABEL_TOPIC = '/puzzlebot_vision/traffic_signals/prediction'

CMD_VEL = "/cmd_vel"
CMD_VEL_GO_2_GOAL = "/cmd_vel/go_2_goal"
CMD_VEL_LINE_DETECT = "/cmd_vel/line_detect"

class Controller():
    def __init__(self):
        self.pose2d = Pose2D()
        self.pose2d.x = 0.0
        self.pose2d.y = 0.0
        self.pose2d.theta = 0.0

        rospy.init_node('puzzlebot_run')

        ##################

        self.client = actionlib.SimpleActionClient('go2pose', GoToPoseAction)

        ##################

        self.client.wait_for_server()

        self.dist = 0.5

        ##################

        self.poseSub = rospy.Subscriber("/pose2d",Pose2D,self.poseCallback)
        self.cmd_vel_go_2_goal_pub = rospy.Publisher(CMD_VEL_GO_2_GOAL,Twist,self.go_2_goal_callback)
        self.cmd_vel_line_detect_pub = rospy.Publisher(CMD_VEL_LINE_DETECT,Twist,self.line_detect_callback)

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
    
    def run(self):
        goal_pose = Pose2D()
        goal_pose.x = 0.0
        goal_pose.y = 0.0
        goal_pose.theta = 0.0
        curr_pose = Pose2D()
        curr_pose.x = self.pose2d.x
        curr_pose.y = self.pose2d.y
        curr_pose.theta = self.pose2d.theta

        STATE = 0

        success = False

        while not success:
            if STATE == 0:
                goal_pose.x = curr_pose.x + self.dist
                goal_pose.y = curr_pose.y
                goal_pose.theta = curr_pose.theta
            elif STATE == 1:
                goal_pose.x = curr_pose.x
                goal_pose.y = curr_pose.y + (self.dist / 2.0)
                goal_pose.theta = curr_pose.theta - 90
            else:
                goal_pose.x = curr_pose.x + (self.dist / 2.0)
                goal_pose.y = curr_pose.y
                goal_pose.theta = curr_pose.theta

            goal = GoToPoseGoal(goal_pose2d = goal_pose)
            self.client.send_goal(goal = goal, feedback_cb = self.callback_feedback, active_cb = self.callback_active)
            self.client.wait_for_result()
            result = self.client.get_result()

            if result.success:
                if STATE < 2:
                    STATE += 1
                else:
                    STATE = 0

    def redLightFlag_callback(self, msg):
        self.redFlag = msg.data
        rospy.loginfo("Red: %s",self.redFlag)

    def greenLightFlag_callback(self, msg):
        self.greenFlag = msg.data
        rospy.loginfo("Green: %s",self.greenFlag)

    def poseCallback(self, data):
        self.pose2d = data

    def callback_active(self):
        rospy.loginfo("Action server is processing the goal")

    def callback_feedback(self, feedback):
        rospy.loginfo("Feedback:%s" % str(feedback))

if __name__ == '__main__':
    try:
        controller = Controller()
        controller.run()
    except rospy.ROSInterruptException:
        pass

