#!/usr/bin/env sh

ros_version=${1:-"hydro"}


echo "####################################################################################################"
echo "##### Checking and installing dependencies for dynamic_robot_localization_tests ros package"
echo "####################################################################################################"


# required system dependencies
sudo apt-get install coreutils -y
sudo apt-get install git -y
sudo apt-get install libav-tools -y
sudo apt-get install python-wstool -y

# required ros packages
sudo apt-get install ros-${ros_version}-amcl -y
sudo apt-get install ros-${ros_version}-base-local-planner -y
sudo apt-get install ros-${ros_version}-controller-manager -y
sudo apt-get install ros-${ros_version}-cob-linear-nav -y
sudo apt-get install ros-${ros_version}-crsm-slam -y
sudo apt-get install ros-${ros_version}-fake-localization -y
sudo apt-get install ros-${ros_version}-gazebo-plugins -y
sudo apt-get install ros-${ros_version}-gazebo-ros -y
sudo apt-get install ros-${ros_version}-gmapping -y
sudo apt-get install ros-${ros_version}-hector-gazebo-plugins -y
sudo apt-get install ros-${ros_version}-hector-mapping -y
sudo apt-get install ros-${ros_version}-hector-slam -y
sudo apt-get install ros-${ros_version}-hector-trajectory-server -y
sudo apt-get install ros-${ros_version}-hokuyo-node -y
sudo apt-get install ros-${ros_version}-joy -y
sudo apt-get install ros-${ros_version}-joystick-drivers -y
sudo apt-get install ros-${ros_version}-map-server -y
sudo apt-get install ros-${ros_version}-move-base -y
sudo apt-get install ros-${ros_version}-openni-launch -y
sudo apt-get install ros-${ros_version}-openni2-launch -y
sudo apt-get install ros-${ros_version}-pr2-common -y
sudo apt-get install ros-${ros_version}-ps3joy -y
sudo apt-get install ros-${ros_version}-robot-state-publisher -y
sudo apt-get install ros-${ros_version}-pointcloud-to-laserscan -y
sudo apt-get install ros-${ros_version}-robot-pose-ekf -y
sudo apt-get install ros-${ros_version}-ros-control -y
sudo apt-get install ros-${ros_version}-ros-controllers -y
sudo apt-get install ros-${ros_version}-rosbag -y
sudo apt-get install ros-${ros_version}-rostopic -y
sudo apt-get install ros-${ros_version}-rqt-plot -y
sudo apt-get install ros-${ros_version}-rviz -y
sudo apt-get install ros-${ros_version}-tf -y
sudo apt-get install ros-${ros_version}-tf2 -y
sudo apt-get install ros-${ros_version}-tf2-ros -y
sudo apt-get install ros-${ros_version}-teleop-twist-joy -y
sudo apt-get install ros-${ros_version}-teleop-twist-keyboard -y
sudo apt-get install ros-${ros_version}-topic-tools -y
sudo apt-get install ros-${ros_version}-urg-node -y


echo "\n\n"
echo "----------------------------------------------------------------------------------------------------"
echo ">>>>> Installation of dependencies finished"
echo "----------------------------------------------------------------------------------------------------"
