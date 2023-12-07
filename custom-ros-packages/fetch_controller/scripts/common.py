#!/usr/bin/env python
# coding:utf-8
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from fetch_controller.msg import state_status
from cv_basics.msg import cv_results

class fetchControl:
    def __init__(self):
        self.pub_status = rospy.Publisher('/state_status', state_status, queue_size=3)
        self.sub_image_results = rospy.Subscriber('/', cv_results, self.JoyStateCallback)

    def cancel(self):
        self.pub_vel.publish(Twist())
        self.pub_vel.unregister()
        self.sub_JoyState.unregister()
        rospy.loginfo("Shutting down this node.")