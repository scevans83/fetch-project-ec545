# Fetch!

This is a repo contains code the final project entitled "Fetch" for EC545 Cyberphysical Systems at Boston University during the Fall 2023 semester. The goal of this project is to design a controller for the Rosmaster X3 to play a game of fetch, where it searches for and finds a ball in the environment, before searching for and return the ball to a goal location. The Rosdmaster X3 uses its onboard camera to perform color detection. The ball is assumed to be red and the goal marker is assumed to be blue for the purpose of this design. 

## Project Structure

- The `InitialFunctionality` directory contains proofs-of-concept, tests, and other functions in progress.
- The `custom-ros-packages` directory contains the four ROS melodic compatible packages that implement the fetch controller. (NOTE: this does not contain all the necessary dependent packages, which can be found on the rosmaster x3 setup page [here](http://www.yahboom.net/study/ROSMASTER-X3))

