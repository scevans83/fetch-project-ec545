#!/usr/bin/env python
# coding:utf-8
import rospy
# from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from fetch_controller.msg import state_status
from cv_basics.msg import cv_results

class fetchControl:
    def __init__(self):
        rospy.on_shutdown(self.cancel)
        self.pub_status = rospy.Publisher('/state_status', state_status, queue_size=3)
        self.sub_image_results = rospy.Subscriber('/video_processing_results', cv_results, self.cv_results_callback)
        self.sub_image_results = rospy.Subscriber('/reached_ball', Bool, self.reached_ball_callback)
        self.most_recent_color_detection = ""
        self.curr_state = ""
        self.reached_ball = False

    def cv_results_callback(self, msg):
      #  rospy.loginfo("controller: updating state with color " + msg.detected_color)
       self.most_recent_color_detection = msg.detected_color
       return
    
    def reached_ball_callback(self, msg):


       
      if "aligning" in self.curr_state:
         rospy.loginfo("controller: received ball_reached")
         self.reached_ball = msg
      else: 
         self.reached_ball = False


       
       

    #TODO: set this up so that it stops the bot (hopefully?)
    def cancel(self):
        # self.pub_vel.publish(Twist())
        self.pub_status.unregister()
        self.sub_image_results.unregister()
        rospy.loginfo("Shutting down this node.")
  
def publish_message(controller):
 
 
  # Tells rospy the name of the node.
  # Anonymous = True makes sure the node has a unique name. Random
  # numbers are added to the end of the name.
  rospy.init_node('state_controller', anonymous=True)
     
  # Go through the loop 10 times per second
  rate = rospy.Rate(10) # 10hz
     

 
  # While ROS is still running.
  while not rospy.is_shutdown():
      
      #very basic current state loop
      curr_status = state_status()


      #### STATE TRANSITION #####

      #initial state def
      if controller.curr_state == "":
         next_state = "exploring1"
         next_color = "red"


      if controller.curr_state == "exploring1":
         rospy.loginfo("controller: attempting state update for exploring1. Most recent detection is %s", controller.most_recent_color_detection)
         next_state = "exploring1" if controller.most_recent_color_detection != "red" else "aligning1"
         next_color = "red"
      

      if controller.curr_state  == "aligning1":
         next_state =  "exploring2" if controller.reached_ball else "aligning1"
         next_color = "red" if not controller.reached_ball else "blue"


      if controller.curr_state  == "exploring2":
         next_state = "exploring2" if controller.most_recent_color_detection != "blue" else "aligning2"
         next_color = "blue"


      if controller.curr_state  == "aligning2":
         next_state = "aligning2" if not controller.reached_ball else "waiting"
         next_color = "blue" if not controller.reached_ball else ""

         #reset this value
         if controller.reached_ball:
            controller.reached_ball = False

      ### end state transition

      #update current state
      controller.curr_state = next_state

      #publish state update
      curr_status.curr_state = controller.curr_state
      curr_status.desired_color_detect = next_color
      controller.pub_status.publish(curr_status)
      rospy.loginfo("controller: state update to %s\n\n", next_state)

      #TODO: left off modifying laser avoidance to use this status to change what it's doing
             
      # Sleep just enough to maintain the desired rate
      rate.sleep()
         
if __name__ == '__main__':
  try:
    controller = fetchControl()
    publish_message(controller)
  except rospy.ROSInterruptException:
    pass