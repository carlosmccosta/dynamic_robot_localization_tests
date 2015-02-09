#!/bin/bash

##################################################
################### parameters ###################
##################################################

results_directory=${1:?'Must specify directory where poin clouds will be saved'}


echo "############################################################################################################################################################"
echo "##### Saving last point clouds to ${results_directory}"
echo "############################################################################################################################################################\n"

rosrun pcl_ros pointcloud_to_pcd input:=/dynamic_robot_localization/ambient_pointcloud _prefix:=${results_directory}/last_ambient_pointcloud_ &
rosrun pcl_ros pointcloud_to_pcd input:=/dynamic_robot_localization/ambient_pointcloud_filtered _prefix:=${results_directory}/last_ambient_pointcloud_filtered_ &
rosrun pcl_ros pointcloud_to_pcd input:=/dynamic_robot_localization/aligned_pointcloud _prefix:=${results_directory}/last_aligned_pointcloud_ &
rosrun pcl_ros pointcloud_to_pcd input:=/dynamic_robot_localization/aligned_pointcloud_inliers _prefix:=${results_directory}/last_aligned_pointcloud_inliers_ &
rosrun pcl_ros pointcloud_to_pcd input:=/dynamic_robot_localization/aligned_pointcloud_outliers _prefix:=${results_directory}/last_aligned_pointcloud_outliers_ &

sleep 20
jobs -l

kill -2 `jobs -p`

sleep 5
jobs -l


echo "\n############################################################################################################################################################"
echo "##### Finished saving point clouds to ${results_directory}"
echo "############################################################################################################################################################\n"
