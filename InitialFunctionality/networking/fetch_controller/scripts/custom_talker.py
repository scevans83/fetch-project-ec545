#!/usr/bin/env python
#following guide here: http://wiki.ros.org/ROS/Tutorials/CustomMessagePublisherSubscriber%28python%29
import rospy
from fetch_controller.msg import controller_state

def talker():
    pub = rospy.Publisher('custom_chatter', controller_state)
    rospy.init_node('custom_talker', anonymous=True)
    r = rospy.Rate(10) #10hz
    msg = controller_state()
    msg.x_position = 5
    msg.y_position = 4
    msg.angle = 180
    msg.test_bool = True

    while not rospy.is_shutdown():
        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass