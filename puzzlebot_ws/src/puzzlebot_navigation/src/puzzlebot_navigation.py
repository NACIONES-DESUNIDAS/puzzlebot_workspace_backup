#!/usr/bin/env python
import rospy
import actionlib
import numpy as np
from math import atan2, pi, sqrt
from geometry_msgs.msg import Pose2D, Twist
from std_msgs.msg import Bool, Float32
from puzzlebot_msgs.msg import GoToPoseAction, GoToPoseFeedback, GoToPoseResult

RATE   =  10

# some nice parameters
linealVel = 0.1
kp = 0.1
kd = 0.1

NAN = np.nan

THETA_THRESHOLD = 10.0 * pi / 180.0
DIST_THRESHOLD = 0.1

ROS_RED_LIGHT_DETECT_TOPIC = '/puzzlebot_vision/traffic_lights/red_light'
ROS_GREEN_LIGHT_DETECT_TOPIC = '/puzzlebot_vision/traffic_lights/green_light'

class Navigator():
    def __init__(self):
        self.pose2d = Pose2D()
        self.pose2d.x = 0.0
        self.pose2d.y = 0.0
        self.pose2d.theta = 0.0

        self.pastAngularError = 0

        ##########################################################################################################
        # TODO: Setup ROS subscribers and publishers, use the callback functions defined bellow if required. 
        #       Your node must subscribe to your previously defined /pose2d named topic, to obtain the current robot's current estimated position info.
        #       YOur node must publish the desired robot's linear and agular velocities using the /cmd_vel topic.
        ##########################################################################################################

        # Your code here...
        self.pub = rospy.Publisher("/cmd_vel",Twist,queue_size=1)
        self.poseRecb = rospy.Subscriber("/pose2d",Pose2D,self.poseCallback)

        ##########################################################################################################

        rospy.init_node('puzzlebot_navigation')

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

        ##########################################################################################################
        # TODO: Add the rospy Subscriber to get the red and green light flags
        ##########################################################################################################

        self.redLightFlag = rospy.Subscriber(ROS_RED_LIGHT_DETECT_TOPIC, Bool, self.redLightFlag_callback)
        self.greenLightFlag = rospy.Subscriber(ROS_GREEN_LIGHT_DETECT_TOPIC, Bool, self.greenLightFlag_callback)

        self.redFlag = Bool()
        self.greenFlag = Bool()

        ##########################################################################################################

        ##########################################################################################################
        # TODO: Setup the Action Server, use adequate message types for the feedback and result of the action.
        #       Use the callback functions defined bellow if required.
        ##########################################################################################################

        # Your code here...
        self.feedback = GoToPoseFeedback()
        self.result = GoToPoseResult()
        self.action = actionlib.SimpleActionServer(name = "go2pose",ActionSpec= GoToPoseAction,execute_cb= self.actionCallback,auto_start=False)

        ##########################################################################################################

        self.angularSub = rospy.Subscriber("/angularError", Float32, self.angularErrorCallback)

        self.action.start()
        rospy.spin()

    def redLightFlag_callback(self, msg):
        self.redFlag = msg.data
        rospy.loginfo("Red: %s",self.redFlag)

    def greenLightFlag_callback(self, msg):
        self.greenFlag = msg.data
        rospy.loginfo("Green: %s",self.greenFlag)

    def poseCallback(self, data):
        self.pose2d = data

    def euclideanDistance(self,xGoal,yGoal,x,y):
        return sqrt((yGoal - y)**2+(xGoal-x)**2)

    def positionalControl(self,x_goal,x_curr,theta_curr,y_goal,y_curr,cmd_vel):
            lastError = 0
            currentError = 0
            # positional control:
            distanceError = self.euclideanDistance(x_goal,x_curr,y_goal,y_curr)
            
            while distanceError > DIST_THRESHOLD:
                # establish a default linear  valocity.
                cmd_vel.linear.x = linealVel
                # calculate angular error
                xDiff = x_goal - x_curr
                yDiff = y_goal - y_curr
                thetaM = atan2(yDiff,xDiff)

                # update error
                lastError = currentError
                currentError = thetaM - theta_curr

                # establish angular velocity
                controlAngularSpeed = kp * currentError + kd * (currentError-lastError)
                # control angular velocity saturation
                cmd_vel.angular.z = controlAngularSpeed if controlAngularSpeed <= 0.3 else 0.3
                self.pub.publish(cmd_vel)
                # compute distance:
                distanceError = self.euclideanDistance(x_goal,x_curr,y_goal,y_curr)

    def orientationControl(self,theta_goal,theta_curr,cmd_vel):
        lastError = 0
        # orientationControl 
        angularError = theta_goal - theta_curr
        while angularError > THETA_THRESHOLD:
            controlAngularSpeed = kp * angularError + kd * angularError
            cmd_vel.linear.x = 0
            cmd_vel.angular.z = controlAngularSpeed if controlAngularSpeed <= 0.3 else 0.3
            self.pub.publish(cmd_vel)
            rospy.loginfo(angularError)
            lastError = angularError
            angularError = theta_goal - theta_curr

    def resetCOmmand(self,cmd_vel):
        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.0
        cmd_vel.linear.z = 0.0
        cmd_vel.angular.x = 0.0
        cmd_vel.angular.y = 0.0
        cmd_vel.angular.z = 0.0
        self.pub.publish(cmd_vel)

    def angularErrorCallback(self, msg):
        rospy.loginfo(msg.data)
        self.rate = rospy.Rate(self.pub_rate)
        while not msg.data == NAN:
            if rospy.is_shutdown():
                break
            angularError = msg.data
            angularErrorAbs = abs(angularError)
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.05
            cmd_vel.linear.y = 0.0
            cmd_vel.linear.z = 0.0
            cmd_vel.angular.x = 0.0
            cmd_vel.angular.y = 0.0
            cmd_vel.angular.z = 0.0

            controlAngularSpeed = kp * angularErrorAbs + kd * (angularErrorAbs-self.pastAngularError)
            # Control angular velocity saturation
            # Factor = -1 if angularError > pi else 1
            if angularErrorAbs > pi and  angularError > 0:
                factor = -1
            elif angularErrorAbs > pi and  angularError < 0:
                factor = 1
            elif angularErrorAbs < pi and angularError < 0:
                factor = -1
            else: 
                factor = 1

            self.pastAngularError = angularErrorAbs

            cmd_vel.angular.z = factor * controlAngularSpeed if controlAngularSpeed <= 0.3 else 0.3 * factor
            
            rospy.loginfo(cmd_vel)

            self.pub.publish(cmd_vel)

    def actionCallback(self, goal):

        ##########################################################################################################
        # TODO: Fill the goal pose data
        ##########################################################################################################

        # Your code here...
        x_goal = goal.goal_pose2d.x
        y_goal = goal.goal_pose2d.y
        theta_goal = goal.goal_pose2d.theta

        ##########################################################################################################

        self.rate = rospy.Rate(self.pub_rate)
        success = False

        STATE = 0

        # positiona angular error
        angularErrorM = 3
        pastAngularErrorM = 3

        # orientation angular error
        angularError = 3
        pastAngularError = 3

        while not success:
            if rospy.is_shutdown():
                break
            # If Action is canceled by other node, stop.
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

            ##########################################################################################################
            # TODO: Use the goal and current robot pose to modify the robot's linear and angular velocity.
            #       You shall only modify the linear.x and angular.z velocities.
            #       A State Machine oriented solution is recommended.
            #       Use the distance and theta thresholds to stop the vehicle trajectory or change between states.
            ##########################################################################################################

            # Your code here...

            if STATE == 0:
                #rospy.loginfo("State: %s | AngularErrorM %s",STATE,angularErrorM)
                cmd_vel.linear.x = 0
                # calculate angular error
                xDiff = x_goal - x_curr
                yDiff = y_goal - y_curr
                thetaM = atan2(yDiff,xDiff)
                # update error
                pastAngularErrorM = angularErrorM
                diff = thetaM - theta_curr
                angularErrorM = abs(diff)

                # establish angular velocity
                controlAngularSpeed = kp * angularErrorM + kd * (angularErrorM-pastAngularErrorM)
                # control angular velocity saturation
                #factor = -1 if angularErrorM > pi else 1
                if angularErrorM > pi and  diff > 0:
                    factor = -1
                elif angularErrorM > pi and  diff < 0:
                    factor = 1
                elif angularErrorM < pi and diff < 0:
                    factor = -1
                else: 
                    factor = 1

                cmd_vel.angular.z = factor * controlAngularSpeed if controlAngularSpeed <= 0.3 else 0.3 * factor
                self.pub.publish(cmd_vel)
                # compute distance:
                if angularErrorM < THETA_THRESHOLD:
                    STATE += 1 
                    self.resetCOmmand(cmd_vel)        
            
            elif STATE == 1:
                """
                if self.redFlag == True:
                    while True:
                        self.resetCOmmand(cmd_vel)
                        if rospy.is_shutdown() or self.greenFlag == True:
                            cmd_vel.linear.x = self.linealVel
                            self.pub.publish(cmd_vel)
                            break
                """

                # calculate angular error
                xDiff = x_goal - x_curr
                yDiff = y_goal - y_curr
                thetaM = atan2(yDiff,xDiff)
                # update error
                pastAngularErrorM = angularErrorM
                diff = thetaM - theta_curr
                angularErrorM = abs(diff)

                # establish angular velocity
                controlAngularSpeed = kp * angularErrorM + kd * (angularErrorM-pastAngularErrorM)
                # control angular velocity saturation
                if angularErrorM > pi and  diff > 0:
                    factor = -1
                elif angularErrorM > pi and  diff < 0:
                    factor = 1
                elif angularErrorM < pi and diff < 0:
                    factor = -1
                else: 
                    factor = 1

                if self.redFlag:
                    #self.resetCOmmand(cmd_vel)
                    
                    while self.greenFlag == False:
                        rospy.loginfo("Enter Red Stop")
                        cmd_vel.linear.x = 0.0
                        self.pub.publish(cmd_vel)
                    cmd_vel.linear.x = self.linealVel
                    self.pub.publish(cmd_vel)
                    rospy.loginfo("Enter Red Stop")

                        #self.resetCOmmand(cmd_vel)
                        #if self.greenFlag and not self.redFlag:
                            #cmd_vel.linear.x = linealVel
                            #self.pub(cmd_vel)
                            #break
                    
                #elif self.greenFlag and not self.redFlag:
                    #cmd_vel.linear.x = linealVel
                else:
                    cmd_vel.angular.z = factor * controlAngularSpeed if controlAngularSpeed <= 0.3 else 0.3 * factor
                    cmd_vel.linear.x = linealVel
                
                self.pub.publish(cmd_vel)
                # compute distance:
                distanceError = self.euclideanDistance(x_goal,y_goal,x_curr,y_curr)
                #rospy.loginfo("State: %s | Distance Error %s",STATE,distanceError)
                if distanceError < DIST_THRESHOLD:
                    STATE += 1
                    self.resetCOmmand(cmd_vel)
                    
            else:
                pastAngularError = angularError
                diff = theta_goal - theta_curr
                angularError = abs(diff)
                #rospy.loginfo("State: %s | Orientation Error %s",STATE,angularError)
                controlAngularSpeed2 = kp * angularError + kd * (angularError-pastAngularError)
                cmd_vel.linear.x = 0

                if angularError > pi and  diff > 0:
                    factor = -1
                elif angularError > pi and  diff < 0:
                    factor = 1
                elif angularError < pi and diff < 0:
                    factor = -1
                else: 
                    factor = 1

                cmd_vel.angular.z = controlAngularSpeed2 * factor if controlAngularSpeed2 <= 0.3 else 0.3 * factor
                self.pub.publish(cmd_vel)
                rospy.loginfo(angularError)
                if angularError < THETA_THRESHOLD:
                    self.resetCOmmand(cmd_vel)
                    success = True
                    STATE = 0

            ##########################################################################################################

            #self.pub.publish(cmd_vel)

            self.feedback.current_pose2d = self.pose2d
            self.action.publish_feedback(self.feedback)
            self.rate.sleep()

        if success:
            self.result.success = True
            self.action.set_succeeded(self.result)
            #self.resetCOmmand(cmd_vel)

if __name__ == '__main__':
    try:
        navigator = Navigator()
    except rospy.ROSInterruptException:
        pass#!/usr/bin/env python
import rospy
import actionlib
import numpy as np
from math import atan2, pi, sqrt
from geometry_msgs.msg import Pose2D, Twist
from std_msgs.msg import Bool, Float32
from puzzlebot_msgs.msg import GoToPoseAction, GoToPoseFeedback, GoToPoseResult

RATE   =  10

# some nice parameters
linealVel = 0.1
kp = 0.1
kd = 0.1

NAN = np.nan

THETA_THRESHOLD = 10.0 * pi / 180.0
DIST_THRESHOLD = 0.1

ROS_RED_LIGHT_DETECT_TOPIC = '/puzzlebot_vision/traffic_lights/red_light'
ROS_GREEN_LIGHT_DETECT_TOPIC = '/puzzlebot_vision/traffic_lights/green_light'

class Navigator():
    def __init__(self):
        self.pose2d = Pose2D()
        self.pose2d.x = 0.0
        self.pose2d.y = 0.0
        self.pose2d.theta = 0.0

        self.pastAngularError = 0

        ##########################################################################################################
        # TODO: Setup ROS subscribers and publishers, use the callback functions defined bellow if required. 
        #       Your node must subscribe to your previously defined /pose2d named topic, to obtain the current robot's current estimated position info.
        #       YOur node must publish the desired robot's linear and agular velocities using the /cmd_vel topic.
        ##########################################################################################################

        # Your code here...
        self.pub = rospy.Publisher("/cmd_vel",Twist,queue_size=1)
        self.poseRecb = rospy.Subscriber("/pose2d",Pose2D,self.poseCallback)

        ##########################################################################################################

        rospy.init_node('puzzlebot_navigation')

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

        ##########################################################################################################
        # TODO: Add the rospy Subscriber to get the red and green light flags
        ##########################################################################################################

        self.redLightFlag = rospy.Subscriber(ROS_RED_LIGHT_DETECT_TOPIC, Bool, self.redLightFlag_callback)
        self.greenLightFlag = rospy.Subscriber(ROS_GREEN_LIGHT_DETECT_TOPIC, Bool, self.greenLightFlag_callback)

        self.redFlag = Bool()
        self.greenFlag = Bool()

        ##########################################################################################################

        ##########################################################################################################
        # TODO: Setup the Action Server, use adequate message types for the feedback and result of the action.
        #       Use the callback functions defined bellow if required.
        ##########################################################################################################

        # Your code here...
        self.feedback = GoToPoseFeedback()
        self.result = GoToPoseResult()
        self.action = actionlib.SimpleActionServer(name = "go2pose",ActionSpec= GoToPoseAction,execute_cb= self.actionCallback,auto_start=False)

        ##########################################################################################################

        self.angularSub = rospy.Subscriber("/angularError", Float32, self.angularErrorCallback)

        self.action.start()
        rospy.spin()

    def redLightFlag_callback(self, msg):
        self.redFlag = msg.data
        rospy.loginfo("Red: %s",self.redFlag)

    def greenLightFlag_callback(self, msg):
        self.greenFlag = msg.data
        rospy.loginfo("Green: %s",self.greenFlag)

    def poseCallback(self, data):
        self.pose2d = data

    def euclideanDistance(self,xGoal,yGoal,x,y):
        return sqrt((yGoal - y)**2+(xGoal-x)**2)

    def positionalControl(self,x_goal,x_curr,theta_curr,y_goal,y_curr,cmd_vel):
            lastError = 0
            currentError = 0
            # positional control:
            distanceError = self.euclideanDistance(x_goal,x_curr,y_goal,y_curr)
            
            while distanceError > DIST_THRESHOLD:
                # establish a default linear  valocity.
                cmd_vel.linear.x = linealVel
                # calculate angular error
                xDiff = x_goal - x_curr
                yDiff = y_goal - y_curr
                thetaM = atan2(yDiff,xDiff)

                # update error
                lastError = currentError
                currentError = thetaM - theta_curr

                # establish angular velocity
                controlAngularSpeed = kp * currentError + kd * (currentError-lastError)
                # control angular velocity saturation
                cmd_vel.angular.z = controlAngularSpeed if controlAngularSpeed <= 0.3 else 0.3
                self.pub.publish(cmd_vel)
                # compute distance:
                distanceError = self.euclideanDistance(x_goal,x_curr,y_goal,y_curr)

    def orientationControl(self,theta_goal,theta_curr,cmd_vel):
        lastError = 0
        # orientationControl 
        angularError = theta_goal - theta_curr
        while angularError > THETA_THRESHOLD:
            controlAngularSpeed = kp * angularError + kd * angularError
            cmd_vel.linear.x = 0
            cmd_vel.angular.z = controlAngularSpeed if controlAngularSpeed <= 0.3 else 0.3
            self.pub.publish(cmd_vel)
            rospy.loginfo(angularError)
            lastError = angularError
            angularError = theta_goal - theta_curr

    def resetCOmmand(self,cmd_vel):
        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.0
        cmd_vel.linear.z = 0.0
        cmd_vel.angular.x = 0.0
        cmd_vel.angular.y = 0.0
        cmd_vel.angular.z = 0.0
        self.pub.publish(cmd_vel)

    def angularErrorCallback(self, msg):
        self.rate = rospy.Rate(self.pub_rate)
        while not msg == NAN:
            if rospy.is_shutdown():
                break
            angularError = msg
            angularErrorAbs = abs(angularError)
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.3
            cmd_vel.linear.y = 0.0
            cmd_vel.linear.z = 0.0
            cmd_vel.angular.x = 0.0
            cmd_vel.angular.y = 0.0
            cmd_vel.angular.z = 0.0

            controlAngularSpeed = kp * angularErrorAbs + kd * (angularErrorAbs-self.pastAngularError)
            # Control angular velocity saturation
            # Factor = -1 if angularError > pi else 1
            if angularErrorAbs > pi and  angularError > 0:
                factor = -1
            elif angularErrorAbs > pi and  angularError < 0:
                factor = 1
            elif angularErrorAbs < pi and angularError < 0:
                factor = -1
            else: 
                factor = 1

            self.pastAngularError = angularErrorAbs

            cmd_vel.angular.z = factor * controlAngularSpeed if controlAngularSpeed <= 0.3 else 0.3 * factor

            self.pub.publish(cmd_vel)

    def actionCallback(self, goal):

        ##########################################################################################################
        # TODO: Fill the goal pose data
        ##########################################################################################################

        # Your code here...
        x_goal = goal.goal_pose2d.x
        y_goal = goal.goal_pose2d.y
        theta_goal = goal.goal_pose2d.theta

        ##########################################################################################################

        self.rate = rospy.Rate(self.pub_rate)
        success = False

        STATE = 0

        # positiona angular error
        angularErrorM = 3
        pastAngularErrorM = 3

        # orientation angular error
        angularError = 3
        pastAngularError = 3

        while not success:
            if rospy.is_shutdown():
                break
            # If Action is canceled by other node, stop.
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

            ##########################################################################################################
            # TODO: Use the goal and current robot pose to modify the robot's linear and angular velocity.
            #       You shall only modify the linear.x and angular.z velocities.
            #       A State Machine oriented solution is recommended.
            #       Use the distance and theta thresholds to stop the vehicle trajectory or change between states.
            ##########################################################################################################

            # Your code here...

            if STATE == 0:
                #rospy.loginfo("State: %s | AngularErrorM %s",STATE,angularErrorM)
                cmd_vel.linear.x = 0
                # calculate angular error
                xDiff = x_goal - x_curr
                yDiff = y_goal - y_curr
                thetaM = atan2(yDiff,xDiff)
                # update error
                pastAngularErrorM = angularErrorM
                diff = thetaM - theta_curr
                angularErrorM = abs(diff)

                # establish angular velocity
                controlAngularSpeed = kp * angularErrorM + kd * (angularErrorM-pastAngularErrorM)
                # control angular velocity saturation
                #factor = -1 if angularErrorM > pi else 1
                if angularErrorM > pi and  diff > 0:
                    factor = -1
                elif angularErrorM > pi and  diff < 0:
                    factor = 1
                elif angularErrorM < pi and diff < 0:
                    factor = -1
                else: 
                    factor = 1

                cmd_vel.angular.z = factor * controlAngularSpeed if controlAngularSpeed <= 0.3 else 0.3 * factor
                self.pub.publish(cmd_vel)
                # compute distance:
                if angularErrorM < THETA_THRESHOLD:
                    STATE += 1 
                    self.resetCOmmand(cmd_vel)        
            
            elif STATE == 1:
                """
                if self.redFlag == True:
                    while True:
                        self.resetCOmmand(cmd_vel)
                        if rospy.is_shutdown() or self.greenFlag == True:
                            cmd_vel.linear.x = self.linealVel
                            self.pub.publish(cmd_vel)
                            break
                """

                # calculate angular error
                xDiff = x_goal - x_curr
                yDiff = y_goal - y_curr
                thetaM = atan2(yDiff,xDiff)
                # update error
                pastAngularErrorM = angularErrorM
                diff = thetaM - theta_curr
                angularErrorM = abs(diff)

                # establish angular velocity
                controlAngularSpeed = kp * angularErrorM + kd * (angularErrorM-pastAngularErrorM)
                # control angular velocity saturation
                if angularErrorM > pi and  diff > 0:
                    factor = -1
                elif angularErrorM > pi and  diff < 0:
                    factor = 1
                elif angularErrorM < pi and diff < 0:
                    factor = -1
                else: 
                    factor = 1

                if self.redFlag:
                    #self.resetCOmmand(cmd_vel)
                    
                    while self.greenFlag == False:
                        rospy.loginfo("Enter Red Stop")
                        cmd_vel.linear.x = 0.0
                        self.pub.publish(cmd_vel)
                    cmd_vel.linear.x = self.linealVel
                    self.pub.publish(cmd_vel)
                    rospy.loginfo("Enter Red Stop")

                        #self.resetCOmmand(cmd_vel)
                        #if self.greenFlag and not self.redFlag:
                            #cmd_vel.linear.x = linealVel
                            #self.pub(cmd_vel)
                            #break
                    
                #elif self.greenFlag and not self.redFlag:
                    #cmd_vel.linear.x = linealVel
                else:
                    cmd_vel.angular.z = factor * controlAngularSpeed if controlAngularSpeed <= 0.3 else 0.3 * factor
                    cmd_vel.linear.x = linealVel
                
                self.pub.publish(cmd_vel)
                # compute distance:
                distanceError = self.euclideanDistance(x_goal,y_goal,x_curr,y_curr)
                #rospy.loginfo("State: %s | Distance Error %s",STATE,distanceError)
                if distanceError < DIST_THRESHOLD:
                    STATE += 1
                    self.resetCOmmand(cmd_vel)
                    
            else:
                pastAngularError = angularError
                diff = theta_goal - theta_curr
                angularError = abs(diff)
                #rospy.loginfo("State: %s | Orientation Error %s",STATE,angularError)
                controlAngularSpeed2 = kp * angularError + kd * (angularError-pastAngularError)
                cmd_vel.linear.x = 0

                if angularError > pi and  diff > 0:
                    factor = -1
                elif angularError > pi and  diff < 0:
                    factor = 1
                elif angularError < pi and diff < 0:
                    factor = -1
                else: 
                    factor = 1

                cmd_vel.angular.z = controlAngularSpeed2 * factor if controlAngularSpeed2 <= 0.3 else 0.3 * factor
                self.pub.publish(cmd_vel)
                rospy.loginfo(angularError)
                if angularError < THETA_THRESHOLD:
                    self.resetCOmmand(cmd_vel)
                    success = True
                    STATE = 0

            ##########################################################################################################

            #self.pub.publish(cmd_vel)

            self.feedback.current_pose2d = self.pose2d
            self.action.publish_feedback(self.feedback)
            self.rate.sleep()

        if success:
            self.result.success = True
            self.action.set_succeeded(self.result)
            #self.resetCOmmand(cmd_vel)

if __name__ == '__main__':
    try:
        navigator = Navigator()
    except rospy.ROSInterruptException:
        pass