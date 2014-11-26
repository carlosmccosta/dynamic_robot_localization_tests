Dynamic Robot Localization tests
================================

# Overview

The dynamic_robot_localization_tests is a ROS package that aims to test the ROS [dynamic_robot_localization package.](https://github.com/carlosmccosta/dynamic_robot_localization)

The testing configurations are managed in [localization_tests.launch](https://github.com/carlosmccosta/dynamic_robot_localization_tests/blob/hydro-devel/launch/localization_tests.launch)

They can use live data from the Gazebo simulator or using [rosbags](https://github.com/carlosmccosta/dynamic_robot_localization_tests/tree/hydro-devel/datasets)

The test results along with environment screenshots / videos are available in [this shared folder](https://www.dropbox.com/sh/nwb6gezj2dan187/AABM2u4BGd12lN__nYFwSktLa?dl=0)



# Testing platforms

The localization system was tested on laser sensor data retrieved from two different robots and executed on the same computer in order to allow direct comparison of computation time. This computer was a Clevo P370EM3 laptop with a Intel Core i7 3630QM CPU at 2.4GHz, 16 GB of RAM DDR3, NVidia GTX680M graphics card and a Samsung 840 Pro SSD.
The sensor data was recorded into rosbags, and is publicly available in the [datasets folder](https://github.com/carlosmccosta/dynamic_robot_localization_tests/tree/hydro-devel/datasets).
The hardware specifications of the lasers used along with all the detailed results and experiments videos are available in [this shared folder](https://www.dropbox.com/sh/nwb6gezj2dan187/AABM2u4BGd12lN__nYFwSktLa?dl=0)

## Jarvis platform
The Jarvis platform is an autonomous ground vehicle equipped with a SICK NAV350 laser for self-localization (mounted about 2 meters from the floor) and a SICK S3000 laser for collision avoidance (mounted about 0.20 meters from the floor). It uses a tricycle locomotion system with two back wheels and a steerable wheel at the front. The 3 DoF ground truth is provided by the SICK NAV350 system using 6 laser reflectors and is certified for robot docking operations with precision up to 4 millimeters.

## Guardian platform
The Guardian platform is an autonomous mobile manipulator equipped with a Hokuyo URG-04LX laser in the front and a Hokuyo URG-04LX_UG01 laser in the back (both mounted about 0.37 meters from the ground). The front laser has a tilting platform which allows 3D mapping of the environment. For locomotion it uses a differential drive system and can be moved with wheels or with tracks. This platform didn’t have a certified ground truth and as such the results of the performed tests could not be quantified with a certified external localization system (the results performed in the Gazebo simulator are available instead).



# Testing environments

The localization system was tested in different variations of two main environments.

## Jarvis in robocup field
The robocup field is a large room with 20.5 meters of length and 7.7 meters of depth. It has two doors, several small windows and two large glass openings into the hallway.
Several tests were performed with the robot at different speeds in this environment but the ground truth was only reliable at low paces. For velocities greater than 5 cm/s the localization system managed to track the robot pose with more accuracy than the ground truth. As such, only the results of the robot moving at 5 cm/s will are available (along with the tests performed in the Stage simulator). These tests were performed with two different movement paths. The first is a simple rounded path that aimed to test the robot in the region of space that had better ground truth (due to its position in relation to the laser reflectors). The second path was more complex and contained several sub paths with different velocities and shapes.

## Guardian in ship interior
The ship interior environment simulated in Gazebo is a large room with 12.4 meters of length and 8.4 meters of depth. It has 4 doors, several small windows and the walls have small ledges at regular intervals.
Given that the Guardian mobile manipulator is expected to work on the walls of this environment, several tests were devised with a path following the lower and right wall of the environment.
The first test was done in a static environment clear of unknown objects and was meant to evaluate the best precision that the localization system could achieve. The second test was done in a cluttered environment and was designed to test the robustness of the localization system against static unknown objects. The last test added a moving car to the scene and aimed to assess the impact of dynamic objects on the point cloud registration algorithms.



# List of related git repositories:

* [dynamic_robot_localization](https://github.com/carlosmccosta/dynamic_robot_localization)
* [pose_to_tf_publisher](https://github.com/carlosmccosta/pose_to_tf_publisher)
* [laserscan_to_pointcloud](https://github.com/carlosmccosta/laserscan_to_pointcloud)
* [mesh_to_pointcloud](https://github.com/carlosmccosta/mesh_to_pointcloud)
* [robot_localization_tools](https://github.com/carlosmccosta/robot_localization_tools)
* [octomap_mapping](https://github.com/carlosmccosta/octomap_mapping)



# More info

* [ICIT 2015 paper](https://www.dropbox.com/sh/yizj93xtvsapl9e/AABdCPKrMX2V58vzpzECKiExa?dl=0)
* [Results folder](https://www.dropbox.com/sh/nwb6gezj2dan187/AABM2u4BGd12lN__nYFwSktLa?dl=0)
* [Dissertation webpage](http://carlosmccosta.wix.com/personal-webpage#!dissertation/c12dl)
* [Dissertation abstract](http://1drv.ms/1odZRYO)
* [Research](http://1drv.ms/1l8yGei)
