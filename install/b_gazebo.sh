#!/usr/bin/env sh

ros_version=${1:-"$(rosversion -d)"}

echo "####################################################################################################"
echo "##### Installing gazebo (http://gazebosim.org/wiki/1.9/install)"
echo "####################################################################################################"


echo "\n\n"
echo "------------------------------------------------"
echo ">>>>> Setting up sources.list"
echo "------------------------------------------------"

sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/gazebo-latest.list'


echo "\n\n"
echo "------------------------------------------------"
echo ">>>>> Setting keys"
echo "------------------------------------------------"
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -



echo "\n\n"
echo "------------------------------------------------"
echo ">>>>> Updating packages index"
echo "------------------------------------------------"
sudo apt-get update



echo "\n\n"
echo "------------------------------------------------"
echo ">>>>> Installing Gazebo"
echo "------------------------------------------------"

sudo apt-get install gazebo -y
sudo apt-get install ros-${ros_version}-gazebo-ros-control -y
sudo apt-get install ros-${ros_version}-gazebo-ros-pkgs -y
sudo apt-get install ros-${ros_version}-gazebo-plugins -y
sudo apt-get install ros-${ros_version}-hector-gazebo-plugins -y


echo "\n\n"
echo "####################################################################################################"
echo "##### Finished"
echo "####################################################################################################"
	