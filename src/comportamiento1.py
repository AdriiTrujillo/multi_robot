#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist

class multi_robots:

    def __init__(self, robot_prefix):
        self.prefix = robot_prefix
        topic = self.prefix + "/mobile_base/commands/velocity"
        self.robot_pub = rospy.Publisher(topic, Twist, queue_size=10)

    def stop(self):
        command = Twist()
        command.linear.x = 0.0
        command.linear.y = 0.0
        command.linear.z = 0.0
        command.angular.x = 0.0
        command.angular.y = 0.0
        command.angular.z = 0.0        
        self.robot_pub.publish(command)

    def go_stright(self):
        command = Twist()
        command.linear.x = 0.3
        command.linear.y = 0.0
        command.linear.z = 0.0
        command.angular.x = 0.0
        command.angular.y = 0.0
        command.angular.z = 0.0
        self.robot_pub.publish(command)

    def turn(self, side = 'right'):
        command = Twist()
        command.linear.x = 0.0
        command.linear.y = 0.0
        command.linear.z = 0.0
        command.angular.x = 0.0
        command.angular.y = 0.0

        if(side == 'right'):
            command.angular.z = 0.65
        elif(side == 'left'):
            command.angular.z = -0.65
        else:
            rospy.loginfo("Side is not valid (right or left)")
            command.angular.z = -0.65

        self.robot_pub.publish(command)

def line_trajectory(robot):
    # To make an line trajectory
    # Come and back
    for i in range(2):
        for i in range(50):
            robot.go_stright()
            rospy.sleep(0.1)
    
        robot.stop()
        rospy.sleep(0.1)

        for i in range(49):
            robot.turn()
            rospy.sleep(0.1)

        robot.stop()
        rospy.sleep(0.1)

    rospy.loginfo("Goal reached ... ")

def main():

    rospy.init_node('Comportamiento_1')

    robot1 = multi_robots('robot1')
    robot2 = multi_robots('robot2')
    robot3 = multi_robots('robot3')
    robot4 = multi_robots('robot4')

    rospy.loginfo("Moving robot1 ...")
    line_trajectory(robot1)
    rospy.loginfo("Moving robot2 ...")
    line_trajectory(robot2)
    rospy.loginfo("Moving robot3 ...")
    line_trajectory(robot3)
    rospy.loginfo("Moving robot4 ...")
    line_trajectory(robot4)


if __name__ == '__main__':
	try: 
		main()
	except rospy.ROSInterruptException: pass