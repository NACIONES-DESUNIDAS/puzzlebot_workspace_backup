#!/usr/bin/env python
import rospy
import actionlib
from math import pi
from geometry_msgs.msg import Pose2D
from puzzlebot_msgs.msg import GoToPoseAction, GoToPoseGoal, GoToPoseFeedback, GoToPoseResult

class Controller():
    def __init__(self):
        # Initialize a rospy node so that the SimpleActionClient can publish and subscribe over ROS.
        rospy.init_node('puzzlebot_run')
        # Creates the SimpleActionClient, passing the type of the action (GoToPoseAction) to the constructor.
        # The name of the Action Server must be provided, in this case 'puzzlebot_navigation'

        ##########################################################################################################
        # TODO: Modify the Action Server Name ('puzzlebot_navigation') with the one you configured in your navigation node
        ##########################################################################################################

        self.client = actionlib.SimpleActionClient('go2pose', GoToPoseAction)

        ##########################################################################################################

        # Waits until the action server has started up and started listening for goals.
        self.client.wait_for_server()

        self.dist = 0.7
    
    def run(self):
        # Initialize a Pose2D object used to send the desired goals to the action server
        goal_pose = Pose2D()
        goal_pose.x = 0.0
        goal_pose.y = 0.0
        goal_pose.theta = 0.0

        # INT variable used for the states definition of the machine state logic.
        STATE = 0

        # Flag used to identify when the robot has completed the desired trajectory and end the while loop.
        success = False

        # Main loop, while the trajectory has not been completed successfully
        while not success:
            # State machine logic used to define multiple goals for the action server, in order to draw a square trajectory
            if STATE == 0:
                goal_pose.x = self.dist
                goal_pose.y = 0.0
                goal_pose.theta = pi / 2.0
            elif STATE == 1:
                goal_pose.x = self.dist
                goal_pose.y = self.dist
                goal_pose.theta = pi
            elif STATE == 2:
                goal_pose.x = 0.0
                goal_pose.y = self.dist
                goal_pose.theta = -pi / 2.0
            else:
                goal_pose.x = 0.0
                goal_pose.y = 0.0
                goal_pose.theta = 0.0

            # Creates a goal to send to the action server.
            goal = GoToPoseGoal(goal_pose2d = goal_pose)
            # Sends the goal to the action server. A Feedback callback is used to log the current action feedback.
            # An active callback is used to log when the action server has begun processing the goal.
            self.client.send_goal(goal = goal, feedback_cb = self.callback_feedback, active_cb = self.callback_active)
            # Waits for the server to finish performing the action.
            self.client.wait_for_result()
            # The result of executing the action is stored to proceed with the state machine logic execution.
            result = self.client.get_result()

            # If the action resulted in success. Change state if not finished or stop the main loop execution.
            if result.success:
                if STATE < 3:
                    STATE += 1
                else:
                    STATE = 0

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

