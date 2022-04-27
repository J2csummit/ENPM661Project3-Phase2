# ENPM661Project3-Phase2
Project 3 Phase 2 for ENPM 661 Path Planning (Part 1 and Part 2)

Project 3 Phase 2
Author: Justin Cheng & Enrico Zoboli
Class: ENPM 661 - Dr. Reza Monfaredi

Task
----
Implement A* Algorithm to find a path between start and end point on a given map for a mobile robot (radius = 10; clearance = 5) with non-holonomic constraints.

Show path planning in 2D enviornment and show results in Gazebo simluation

Used Libraries
--------------
numpy, math, sys, collision, pygame, time, matplotlib, rospy, geometry_msgs

Github Link
-----------
https://github.com/J2csummit/ENPM661Project3-Phase2

Steps to run code
-----------------
1. Ensure that you have Python 3 installed.
	 - a) Code was written using Python 3.7.
2. Ensure you have ROS Melodic or Noetic installed.
3. Ensure that you have python libraries "collisions", "pygame", "numpy", "matplotlib", and "rospkg" installed.
	 - a) "pip install %libraryname%", where %libraryname% is the name of the library to be installed.
4. Extract zip file into catkin_ws/src folder
5. In terminal, navigate to catkin_ws with "cd ~/catkin_ws"
6. In terminal, run "catkin_make"
7. In terminal, run "source devel/setup.bash"
8. In terminal, run "cd ~/catkin_ws/src/proj3/src"
9. In terminal, run "chmod +x drive.py"
10. In terminal, run "roslaunch proj3 run.launch"
11. Program will prompt for multiple inputs. These inputs are: robot low RPM, robot high RPM, robot clearance, starting (x, y, angle), and goal (x, y, angle). To demonstrate the results in the video, input the following values.
	 - a) robot low RPM = 50
	 - b) robot high RPM = 100
	 - c) robot clearance = 10
	 - d) starting (x, y, angle) = (5, 9, 135)
	 - e) goal (x, y, angle) = (4, 1, 45)
	 - f) Output will briefly show 2D simluation in pygame, and then Gazebo will follow by implementing the path with turtlebot
