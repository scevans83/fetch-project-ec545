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
       rospy.loginfo("updating state with color " + msg.detected_color)
       self.most_recent_color_detection = msg.detected_color
       return
    
    def reached_ball_callback(self, msg):
       self.reached_ball = msg
       
       

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
         next_state = "exploring1" if controller.most_recent_color_detection != "red" else "aligning1"
         next_color = "red"
      

      if controller.curr_state  == "aligning1":
         next_state = "aligning1" if not controller.reached_ball else "exploring2"
         next_color = "red" if not controller.reached_ball else "green"


      if controller.curr_state  == "exploring2":
         next_state = "exploring2" if controller.most_recent_color_detection != "green" else "aligning2"
         next_color = "green"


      if controller.curr_state  == "aligning2":
         next_state = "aligning2" if controller.reached_ball != True else "exploring1"
         next_color = "green" if controller.reached_ball != True else "red"

      ### end state transition

      if next_state != controller.curr_state:
         #update current state
         controller.curr_state = next_state

         #publish state update
         curr_status.curr_state = controller.curr_state
         curr_status.desired_color_detect = next_color
         controller.pub_status.publish(curr_status)

      #TODO: left off modifying laser avoidance to use this status to change what it's doing
             
      # Sleep just enough to maintain the desired rate
      rate.sleep()
         
if __name__ == '__main__':
  try:
    controller = fetchControl()
    publish_message(controller)
  except rospy.ROSInterruptException:
    pass