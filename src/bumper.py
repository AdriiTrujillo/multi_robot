#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent 

bump = False

class multi_robots:

    def __init__(self, robot_prefix):
        self.prefix = robot_prefix
        topic = self.prefix + "/mobile_base/commands/velocity"
        self.robot_pub = rospy.Publisher(topic, Twist, queue_size=10)
        bump_topic = self.prefix + "/mobile_base/events/bumper"
        sub = rospy.Subscriber(bump_topic, BumperEvent, self.processBump)

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

    def processBump(self,data):
        global bump
        if (data.state == BumperEvent.PRESSED):
            bump = True
            rospy.loginfo("Bump detected")


def square_trajectory(robot1, robot2):
    # To make an square trajectory
    for i in range(4):
        for i in range(50):
            robot1.go_stright()
            robot2.go_stright()
            rospy.sleep(0.1)
    
        robot1.stop()
        robot2.stop()
        rospy.sleep(0.1)

        for i in range(25):
            robot1.turn()
            robot2.turn()
            rospy.sleep(0.1)

        robot1.stop()
        robot2.stop()
        rospy.sleep(0.1)

    rospy.loginfo("Goal reached ... ")


def line_trajectory(robot1, robot2):
    # To make an line trajectory
    # Come and back
    for i in range(2):
        for i in range(50):
            robot1.go_stright()
            robot2.go_stright()
            rospy.sleep(0.1)
    
        robot1.stop()
        robot2.stop()
        rospy.sleep(0.1)

        for i in range(49):
            robot1.turn()
            robot2.turn()
            rospy.sleep(0.1)

        robot1.stop()
        robot2.stop()
        rospy.sleep(0.1)

    rospy.loginfo("Goal reached ... ")

def main():
    rospy.init_node('multi_robots')
    robot1 = multi_robots('robot1')

    while not rospy.is_shutdown():
        if bump == True: 
            for i in range(49):
                robot1.turn()
                rospy.sleep(0.1)

            bump = False


if __name__ == '__main__':
	try: 
		main()
	except rospy.ROSInterruptException: pass