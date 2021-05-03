#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent 

# bump = False

class multi_robots:

    leftBump_ = False
    centerBump_ = False
    rightBump_ = False

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

    def get_leftBump(self):
        return self.leftBump_

    def get_centerBump(self):
        return self.centerBump_

    def get_rihgtBump(self):
        return self.rightBump_

    def release_bump(self):
        self.leftBump_ = False
        self.centerBump_ = False
        self.rightBump_ = False

    def processBump(self,data):
        global bump
        if (data.state == BumperEvent.LEFT):
            self.leftBump_ = True
            rospy.loginfo("Left Bump detected!")

        if (data.state == BumperEvent.CENTER):
            self.centerBump_ = True
            rospy.loginfo("Center Bump detected!")

        if (data.state == BumperEvent.RIGHT):
            self.rightBump_ = True
            rospy.loginfo("RIGHT Bump detected!")


def detect_obstacles(robot):

    if robot.get_centerBump(): 
        for i in range(49):
            robot.turn()
            rospy.sleep(0.1)
        # robot.release_center()
        robot.release_bump()

    elif robot.get_leftBump(): 
        for i in range(35):
            robot.turn("left")
            rospy.sleep(0.1)
        robot.release_bump()

    elif robot.get_rihgtBump(): 
        for i in range(35):
            robot.turn()
            rospy.sleep(0.1)
        robot.release_bump()

def main():
    rospy.init_node('Comportamiento_2')
    robot1 = multi_robots('robot1')

    while not rospy.is_shutdown():
        detect_obstacles(robot1)
        robot1.go_stright()
        # rospy.sleep()
            

if __name__ == '__main__':
	try: 
		main()
	except rospy.ROSInterruptException: pass