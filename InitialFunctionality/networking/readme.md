This fetch_controller ros package contains the custom state message for transmitting the state information from the central controller to the rosmaster. 

To use it, create a catkin workspace and add this folder to the src directory, then make the package by calling catkin_make. You can verify that the call worked as expected by calling "source devel/setup.bash" and verifying that the rospackage "fetch_controller" is avaiable.

To setup communication between multiple machines, they need to be on the same LAN. Then follow the steps here: http://wiki.ros.org/ROS/Tutorials/MultipleMachines

Note that it is very important to set the ROS_IP variable on the machine that is publishing a topic, as otherwise no traffic will be detected on the other machine.

Note: I am running this in a docker container. In order for this to function, I needed to add the argument --network=host to my docker build and docker run commands. 
