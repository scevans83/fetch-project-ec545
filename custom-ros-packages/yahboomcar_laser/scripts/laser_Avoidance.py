#!/usr/bin/env python
# coding:utf-8
import math
import cv2
import numpy as np
import time
import random #added KO
from common import *
from time import sleep
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
from dynamic_reconfigure.server import Server
from cv_bridge import CvBridge
from yahboomcar_laser.cfg import laserAvoidPIDConfig
from fetch_controller.msg import state_status
from cv_basics.msg import cv_results
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
        self.cv_sub = rospy.Subscriber('/video_processing_results', cv_results, self.process_cv_results, queue_size=3)
        self.state_status_sub = rospy.Subscriber('/state_status', state_status, self.process_state_updates, queue_size=3)
        self.current_state = "exploring"
        self.desired_color_detection = "red"
        self.aligning_angle = True
        self.in_range_of_goal = False

        #output publisher
        self.aligned_pub = rospy.Publisher('/reached_ball', Bool, queue_size=1)
        # self.EdiPublisher = rospy.Publisher('edition', Float32, queue_size=100)

        #used in aligning
        self.image_center = None
        self.detection_center = None
        self.center_detections = np.zeros(5)
        self.moving_average_initalized = False
        self.last_color_detection = ""
        self.running_error = 0
        self.last_error = 0
        self.velocity_decision = "search"
        self.last_direction = -1
        self.end_state_counter = 0

        

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
    
    def process_cv_results(self, results):

        if self.current_state in ["aligning1", "aligning2"]:

            self.image_center = results.image_center
            self.detection_center = results.contour_center
            self.last_color_detection = results.detected_color

            #intialize the moving average
            if(not self.moving_average_initalized):
                self.center_detections = self.image_center*np.ones(5, dtype=np.int32)
                self.moving_average_initalized = True

            self.center_detections[1:-1] = self.center_detections[0:-2]
            self.center_detections[0] = results.contour_center

        return

    def process_state_updates(self, state):

        #if we are about to transition states stop so we can actually align
        if self.current_state in ["exploring1", "exploring2"] and state.curr_state not in ["exploring1", "exploring2"]:
            rospy.loginfo("stop the boy please")
            twist = Twist()
            self.ros_ctrl.pub_vel.publish(twist)
            sleep(0.2)

        self.current_state = state.curr_state
        self.desired_color_detection = state.desired_color_detect

        return

    def registerScan(self, scan_data):
        if not isinstance(scan_data, LaserScan): return
        # 记录激光扫描并发布最近物体的位置（或指向某点）
        # Record the laser scan and publish the position of the nearest object (or point to a point)
        ranges = np.array(scan_data.ranges)
        self.Right_warning = 0
        self.Left_warning = 0
        self.front_warning = 0

        #handle our alignment case
        if self.current_state in ["aligning1", "aligning2"]:

            rospy.loginfo("aligning the boy")
            max_velocity = 0.5
            kp = 0.5 #currently is random.
            ki = 0.2
            kd = 0.1
            time_step = 1/10 #assume ros 10 hz is met
            curr_error = np.inf


            #we're gonna process the lidar ourselves in the same way they do, but better because these MFs have never seen numpy cook
            angles = np.array((scan_data.angle_min + scan_data.angle_increment * np.arange(len(ranges))) * RAD2DEG)

            right_angles = (angles < 160) & (angles > 180 - self.LaserAngle)
            left_angles =  (angles > -160) & (angles < self.LaserAngle - 180)
            front_angles = np.abs(angles) > 160

            self.Right_warning = np.sum(ranges[right_angles] < self.ResponseDist/2)
            self.Left_warning = np.sum(ranges[left_angles] < self.ResponseDist/2)
            self.front_warning = np.sum(ranges[front_angles] < self.ResponseDist/2)


            if self.velocity_decision == "search" and self.last_color_detection == self.desired_color_detection:
                self.ros_ctrl.pub_vel.publish(Twist())
                sleep(0.2)

             
            if self.last_color_detection == self.desired_color_detection:
                self.velocity_decision = "p control"
                curr_error = (self.image_center - self.detection_center)/np.float(self.image_center*2)
                self.running_error += time_step*curr_error
                angular_velocity = kp*curr_error + ki*self.running_error + (curr_error - self.last_error)*kd
                angular_velocity = np.sign(angular_velocity) * min(np.abs(angular_velocity), max_velocity)
                self.last_error = curr_error
                self.last_direction = np.sign(angular_velocity)

            else:
                self.velocity_decision = "search"
                sign = -1*self.last_direction
                sign = sign if sign != 0 else -1
                angular_velocity = sign*max_velocity
                self.running_error = 0 #reset this so error only reflects when we see the thing

            rospy.loginfo("Setting velocity based on %s.\n Image center is %d.\n Center detection is %d.\n Desired velocity is %f", self.velocity_decision, self.image_center, self.detection_center, angular_velocity)


            twist = Twist()

            #end criteria for angle alignment (first align angle, then get as close as possible)
            angle_error_criteria = np.abs(angular_velocity) < 0.2*max_velocity
            twist.angular.z = 0 if angle_error_criteria or not self.aligning_angle else angular_velocity

            self.aligning_angle = not angle_error_criteria 
            rospy.loginfo("angle alignment state %d", self.aligning_angle)


            #TODO: it isn't moving towards the guy anymore. Figure out why
            rospy.loginfo("number forward lidar detections %d and current velocity driver %s", self.front_warning, self.velocity_decision)
            if not self.aligning_angle: #np.abs(curr_error) <= 0.35 and self.velocity_decision != "search":
                if self.front_warning < 20: #may want to have more ifs later so nesting this way
                    twist.linear.x = 0.1
                    rospy.loginfo("moving towards detection")
                else:
                    twist.linear.x = 0
                    self.in_range_of_goal = True

                #TODO: figure out a way to signal a transition in state

            self.ros_ctrl.pub_vel.publish(twist)
            sleep(0.2)

            #end condition
            if not self.aligning_angle and self.in_range_of_goal:
                #for now, just exit if we get it once
                rospy.loginfo("reached end condition!")
                self.aligned_pub.publish(True)
                self.current_state = "" #ensure we don't do anything until the controller tells us

                #reset a ton of variables
                self.image_center = None
                self.detection_center = None
                self.center_detections = np.zeros(5)
                self.moving_average_initalized = False
                self.last_color_detection = ""
                self.running_error = 0
                self.last_error = 0
                self.velocity_decision = "search"
                self.last_direction = -1
                self.end_state_counter = 0
                self.in_range_of_goal = False
                self.aligning_angle = True


       
        #Treat the orignal lidear avoidance as the exploring state
        if self.current_state in ["exploring1", "exploring2"]:
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
            angles = np.array((scan_data.angle_min + scan_data.angle_increment * np.arange(len(ranges))) * RAD2DEG)
            right_angles = (angles < 160) & (angles > 180 - self.LaserAngle)
            left_angles =  (angles > -160) & (angles < self.LaserAngle - 180)
            front_angles = np.abs(angles) > 160

            Right_warning = np.sum(ranges[right_angles] < self.ResponseDist)
            Left_warning = np.sum(ranges[left_angles] < self.ResponseDist)
            front_warning = np.sum(ranges[front_angles] < self.ResponseDist)

            assert(front_warning == self.front_warning)
            assert(Left_warning == self.Left_warning)
            assert(Right_warning == self.Right_warning)


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
