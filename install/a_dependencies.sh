#!/usr/bin/env sh

ros_version=${1:-"$(rosversion -d)"}
install_args=${2:-"-y --allow-unauthenticated"}


echo "####################################################################################################"
echo "##### Checking and installing dependencies for dynamic_robot_localization_tests ros package"
echo "####################################################################################################"

sudo apt-get update
sudo apt-get upgrade ${install_args}
sudo apt-get dist-upgrade ${install_args}

# required system dependencies
sudo apt-get install coreutils ${install_args}
sudo apt-get install git ${install_args}
sudo apt-get install libav-tools ${install_args}
sudo apt-get install python-wstool ${install_args}

# required ros packages
sudo apt-get install ros-${ros_version}-amcl ${install_args}
sudo apt-get install ros-${ros_version}-base-local-planner ${install_args}
sudo apt-get install ros-${ros_version}-controller-manager ${install_args}
sudo apt-get install ros-${ros_version}-cob-linear-nav ${install_args}
sudo apt-get install ros-${ros_version}-crsm-slam ${install_args}
sudo apt-get install ros-${ros_version}-fake-localization ${install_args}
sudo apt-get install ros-${ros_version}-gazebo-plugins ${install_args}
sudo apt-get install ros-${ros_version}-gazebo-ros ${install_args}
sudo apt-get install ros-${ros_version}-gmapping ${install_args}
sudo apt-get install ros-${ros_version}-hector-gazebo-plugins ${install_args}
sudo apt-get install ros-${ros_version}-hector-mapping ${install_args}
sudo apt-get install ros-${ros_version}-hector-slam ${install_args}
sudo apt-get install ros-${ros_version}-hector-trajectory-server ${install_args}
sudo apt-get install ros-${ros_version}-hokuyo-node ${install_args}
sudo apt-get install ros-${ros_version}-joy ${install_args}
sudo apt-get install ros-${ros_version}-joystick-drivers ${install_args}
sudo apt-get install ros-${ros_version}-map-server ${install_args}
sudo apt-get install ros-${ros_version}-move-base ${install_args}
sudo apt-get install ros-${ros_version}-openni-launch ${install_args}
sudo apt-get install ros-${ros_version}-openni2-launch ${install_args}
sudo apt-get install ros-${ros_version}-pr2-common ${install_args}
sudo apt-get install ros-${ros_version}-ps3joy ${install_args}
sudo apt-get install ros-${ros_version}-robot-state-publisher ${install_args}
sudo apt-get install ros-${ros_version}-pointcloud-to-laserscan ${install_args}
sudo apt-get install ros-${ros_version}-robot-pose-ekf ${install_args}
sudo apt-get install ros-${ros_version}-ros-control ${install_args}
sudo apt-get install ros-${ros_version}-ros-controllers ${install_args}
sudo apt-get install ros-${ros_version}-rosbag ${install_args}
sudo apt-get install ros-${ros_version}-rostopic ${install_args}
sudo apt-get install ros-${ros_version}-rqt-plot ${install_args}
sudo apt-get install ros-${ros_version}-rviz ${install_args}
sudo apt-get install ros-${ros_version}-tf ${install_args}
sudo apt-get install ros-${ros_version}-tf2 ${install_args}
sudo apt-get install ros-${ros_version}-tf2-ros ${install_args}
sudo apt-get install ros-${ros_version}-teleop-twist-joy ${install_args}
sudo apt-get install ros-${ros_version}-teleop-twist-keyboard ${install_args}
sudo apt-get install ros-${ros_version}-topic-tools ${install_args}
sudo apt-get install ros-${ros_version}-urg-node ${install_args}


sudo apt-get upgrade ${install_args}
sudo apt-get dist-upgrade ${install_args}


echo "\n\n"
echo "----------------------------------------------------------------------------------------------------"
echo ">>>>> Installation of dependencies finished"
echo "----------------------------------------------------------------------------------------------------"
