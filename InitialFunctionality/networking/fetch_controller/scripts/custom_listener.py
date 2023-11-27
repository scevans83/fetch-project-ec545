#!/usr/bin/env python
import rospy
from fetch_controller.msg import controller_state

def callback(data):
    rospy.loginfo("x: %d, y: %d, theta: %d, bool: %d" % (data.x_position, data.y_position, data.angle, data.test_bool))

def listener():
    rospy.init_node('custom_listener', anonymous=True)
    rospy.Subscriber("custom_chatter", controller_state, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()