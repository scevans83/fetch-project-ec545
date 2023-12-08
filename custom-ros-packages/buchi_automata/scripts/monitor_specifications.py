#!/usr/bin/env python
# coding:utf-8
import rospy
from std_msgs.msg import Bool
from fetch_controller.msg import state_status
import time

class buchiAutomata:
   def __init__(self):
        rospy.on_shutdown(self.cancel)
      #   self.pub_automata = rospy.Publisher('/automata_output', Bool, queue_size=3)
        self.laser_results = rospy.Subscriber('/reached_ball', Bool, self.reached_ball_callback)
        self.state_sub = rospy.Subscriber('/state_status', state_status, self.state_status_callback)
        self.curr_state = ""
        self.prev_state = ""
        self.reached_ball = False
        self.monitor1 = False
        self.monitor2 = False

    
   def reached_ball_callback(self, msg):
      # rospy.loginfo("automata: checking ball reached")
       
      if "aligning" in self.curr_state:
         self.reached_ball = msg
      else: 
         self.reached_ball = False
   
   def state_status_callback(self, msg):
      # rospy.loginfo("automata: updating state info")
      self.prev_state = self.curr_state
      self.curr_state = msg.curr_state
   

    #TODO: set this up so that it stops the bot (hopefully?)
   def cancel(self):
        self.laser_results.unregister()
        self.state_sub.unregister()
        rospy.loginfo("Shutting down this node.")
  
def monitor_specs(automata_info):
  
 
   # Tells rospy the name of the node.
   # Anonymous = True makes sure the node has a unique name. Random
   # numbers are added to the end of the name.
   rospy.init_node('spec_monitor', anonymous=True)
      
   # Go through the loop 10 times per second
   rate = rospy.Rate(10) # 10hz

      #initialize to values that make sense
   spec1_timer_start = time.time()
   spec2_timer_start = spec1_timer_start
   elapsed_time = 0

   rospy.loginfo("automata: beginning monitoring ")
 
   # # While ROS is still running.
   while not rospy.is_shutdown():
   #    #spec 1 timer start monitor
      if automata_info.curr_state == "exploring1" and not automata_info.monitor1:
         rospy.loginfo("automata: beginning monitor of spec 1: G(exploring1 => F(ball_grabbed))")
         spec1_timer_start = time.time()
         automata_info.monitor1 = True

   #    #spec 2 timer start monitor
      if not automata_info.monitor1 and automata_info.curr_state == "exploring2" and not automata_info.monitor2:
         rospy.loginfo("automata: beginning monitor of spec 2: G(exploring2 => F(at_goal))")
         spec2_timer_start = time.time()
         automata_info.monitor2 = True

      #checking for end condition on monitor 1 (spec 1)
      if automata_info.monitor1:
         if automata_info.reached_ball:
            elapsed_time = time.time() - spec1_timer_start
            automata_info.monitor1 = False
            automata_info.reached_ball = False
            rospy.loginfo("automata: reached end condition of spec 1. Time elapsed: %f s", elapsed_time)
         
      if automata_info.monitor2:
         #note: ball_reached is the same variable for both end points of "align" style states
         #note: this causes dumb bugs 
         if automata_info.reached_ball:
            elapsed_time = time.time() - spec2_timer_start
            automata_info.monitor2 = False
            automata_info.reached_ball = False
            rospy.loginfo("automata: reached end condition of spec 2. Time elapsed: %f s", elapsed_time)
            
      # Sleep just enough to maintain the desired rate
      rate.sleep()
         
if __name__ == '__main__':
  try:
    automata_info = buchiAutomata()
    monitor_specs(automata_info)
  except rospy.ROSInterruptException:
    pass