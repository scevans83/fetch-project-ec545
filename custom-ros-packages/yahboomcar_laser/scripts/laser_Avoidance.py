#!/usr/bin/env python
# coding:utf-8
import math
import cv2
import numpy as np
import time
import random #added KO
from common import *
from time import sleep
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
from dynamic_reconfigure.server import Server
from cv_bridge import CvBridge
from yahboomcar_laser.cfg import laserAvoidPIDConfig
from align_color import find_contours_and_colors
RAD2DEG = 180 / math.pi
direction = [-1, 1]

class laserAvoid:
    def __init__(self):
        rospy.on_shutdown(self.cancel)
        self.r = rospy.Rate(20)
        self.Moving = False
        self.switch = False
        self.Right_warning = 0
        self.Left_warning = 0
        self.front_warning = 0
        self.ros_ctrl = ROSCtrl()
        Server(laserAvoidPIDConfig, self.dynamic_reconfigure_callback)
        self.linear = 0.5
        self.angular = 1.0
        self.ResponseDist = 0.55
        self.LaserAngle = 40  # 10~180
        self.sub_laser = rospy.Subscriber('/scan', LaserScan, self.registerScan, queue_size=1)

        

    def cancel(self):
        self.ros_ctrl.pub_vel.publish(Twist())
        self.ros_ctrl.cancel()
        self.sub_laser.unregister()
        rospy.loginfo("Shutting down this node.")

    def dynamic_reconfigure_callback(self, config, level):
        self.switch = config['switch']
        self.linear = config['linear']
        self.angular = config['angular']
        self.LaserAngle = config['LaserAngle']
        self.ResponseDist = config['ResponseDist']
        return config

    def registerScan(self, scan_data):
        if not isinstance(scan_data, LaserScan): return
        # 记录激光扫描并发布最近物体的位置（或指向某点）
        # Record the laser scan and publish the position of the nearest object (or point to a point)
        ranges = np.array(scan_data.ranges)
        self.Right_warning = 0
        self.Left_warning = 0
        self.front_warning = 0

        ####INESERTIONS###

        # ret, img = self.frame.read()
        # if ret == true:
        #     rospy.loginfo('publishing video frame')
        #     pub.publish(br.cv2_to_imgmsg(frame))
        # red_lower = (0, 0, 175)
        # red_upper = (200, 200, 255)

        # contours, color_list = find_contours_and_colors(img, red_lower, red_upper)

        # cv2.imshow('labeled image',img)
        # action = cv2.waitKey(20)

        # rospy.loginfo("hello")

        # if "red" in color_list:
            

        #             #Do some angular adjustment to track the red contours
        #     num_frames = 5
        #     image_center_x = np.array(np.int32(np.shape(img)[0]/2))
        #     moving_average = np.ones(num_frames)*image_center_x

        #     color_of_interest = "red"
        #     # color_of_interest = "green"
        #     # color_of_interest = "blue"
        #     if color_of_interest in color_list:

        #         #find the max red contour
        #         max_color_idx = None
        #         max_color_area = -np.inf
        #         for idx in range(len(color_list)):
        #             if color_list[idx] == color_of_interest:
        #                 curr_area = cv2.contourArea(contours[idx])
        #                 if curr_area > max_color_area:
        #                     max_color_area = curr_area
        #                     max_color_idx = idx


        #         curr_offset = find_contour_center(contours[max_color_idx])[0]
                


                
        #     else:
        #         curr_offset = image_center_x

        #     #update velocity
        #     moving_average[1:-1] = moving_average[0:-2]
        #     moving_average[0] = curr_offset

        #     #find the center of the max red contour
            

        #     kp = 5 #currenlty is random.
        #     angular_velocity = kp*(image_center_x - np.mean(moving_average))/np.shape(img)[0]

        #     rospy.loginfo("desired velocity: ", angular_velocity)
        #     max_velocity = 1.0
        #     angular_velocity = np.sign(angular_velocity) * min(np.abs(angular_velocity), max_velocity)

        #     cmd = Twist()
        #     cmd.linear.x = 0
        #     cmd.angular.z = angular_velocity
        #     self.ros_ctrl.pub_vel.publish(cmd)

        # ########






        # 按距离排序以检查从较近的点到较远的点是否是真实的东西
        # if we already have a last scan to compare to
        for i in range(len(ranges)):
            angle = (scan_data.angle_min + scan_data.angle_increment * i) * RAD2DEG
            # if angle > 90: print "i: {},angle: {},dist: {}".format(i, angle, scan_data.ranges[i])
            # 通过清除不需要的扇区的数据来保留有效的数据
            if 160 > angle > 180 - self.LaserAngle:
                if ranges[i] < self.ResponseDist: self.Right_warning += 1
            if - 160 < angle < self.LaserAngle - 180:
                if ranges[i] < self.ResponseDist: self.Left_warning += 1
            if abs(angle) > 160:
                if ranges[i] <= self.ResponseDist: self.front_warning += 1
        # print (self.Left_warning, self.front_warning, self.Right_warning)
        if self.ros_ctrl.Joy_active or self.switch == True:
            if self.Moving == True:
                self.ros_ctrl.pub_vel.publish(Twist())
                self.Moving = not self.Moving
            return
        self.Moving = True
        twist = Twist()
        # 左正右负
        # Left positive and right negative
        if self.front_warning > 10 and self.Left_warning > 10 and self.Right_warning > 10:
            # print ('1, there are obstacles in the left and right, turn right')
            twist.linear.x = -0.15
            twist.angular.z = -self.angular
            self.ros_ctrl.pub_vel.publish(twist)
            sleep(0.2)
        elif self.front_warning > 10 and self.Left_warning <= 10 and self.Right_warning > 10:
            # print ('2, there is an obstacle in the middle right, turn left')
            twist.linear.x = 0
            twist.angular.z = self.angular
            self.ros_ctrl.pub_vel.publish(twist)
            sleep(0.2)
            if self.Left_warning > 10 and self.Right_warning <= 10:
                # print ('3, there is an obstacle on the left, turn right')
                twist.linear.x = 0
                twist.angular.z = -self.angular
                self.ros_ctrl.pub_vel.publish(twist)
                sleep(0.5)
        elif self.front_warning > 10 and self.Left_warning > 10 and self.Right_warning <= 10:
            # print ('4. There is an obstacle in the middle left, turn right')
            twist.linear.x = 0
            twist.angular.z = -self.angular
            self.ros_ctrl.pub_vel.publish(twist)
            sleep(0.2)
            if self.Left_warning <= 10 and self.Right_warning > 10:
                # print ('5, there is an obstacle on the left, turn left')
                twist.linear.x = 0
                twist.angular.z = self.angular
                self.ros_ctrl.pub_vel.publish(twist)
                sleep(0.5)
        elif self.front_warning > 10 and self.Left_warning < 10 and self.Right_warning < 10:
            # print ('6, there is an obstacle in the middle, turn left or right')
            twist.linear.x = 0
            twist.angular.z = random.choice(direction)*self.angular
            self.ros_ctrl.pub_vel.publish(twist)
            sleep(0.2)
        elif self.front_warning < 10 and self.Left_warning > 10 and self.Right_warning > 10:
            # print ('7. There are obstacles on the left and right, turn right')
            twist.linear.x = 0
            twist.angular.z = -self.angular
            self.ros_ctrl.pub_vel.publish(twist)
            sleep(0.4)
        elif self.front_warning < 10 and self.Left_warning > 10 and self.Right_warning <= 10:
            # print ('8, there is an obstacle on the left, turn right')
            twist.linear.x = 0
            twist.angular.z = -self.angular
            self.ros_ctrl.pub_vel.publish(twist)
            sleep(0.2)
        elif self.front_warning < 10 and self.Left_warning <= 10 and self.Right_warning > 10:
            # print ('9, there is an obstacle on the right, turn left')
            twist.linear.x = 0
            twist.angular.z = self.angular
            self.ros_ctrl.pub_vel.publish(twist)
            sleep(0.2)
        elif self.front_warning <= 10 and self.Left_warning <= 10 and self.Right_warning <= 10:
            # print ('10, no obstacles, go forward')
            twist.linear.x = self.linear
            twist.angular.z = 0
            self.ros_ctrl.pub_vel.publish(twist)
        self.r.sleep()
        # else : self.ros_ctrl.pub_vel.publish(Twist())


if __name__ == '__main__':
    rospy.init_node('laser_Avoidance', anonymous=False)
    tracker = laserAvoid()
    rospy.spin()
