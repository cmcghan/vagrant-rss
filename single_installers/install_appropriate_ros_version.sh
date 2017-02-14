#!/bin/bash -e
# Copyright by California Institute of Technology, University of Cincinnati
# All rights reserved. See LICENSE file at:
# https://github.com/cmcghan/vagrant-rss

#
# in the initialization process for vagrant, this bash script is run as user 'root' from /vagrant
#

echo "Start of install_appropriate_ros_version.sh script!"
echo "input arguments: ROSVERSION [SCRIPTUSER] [WORKSPACEDIR] [-f]"
echo "(note: optional input arguments in [])"
echo "(note: there is no default ROSVERSION. Acceptable inputs are: indigo jade kinetic)"
echo "(note: default [SCRIPTUSER] is \"vagrant\")"
echo "(note: SCRIPTUSER must be given as an argument for WORKSPACEDIR to be read and accepted from commandline)"
echo "(note: default [WORKSPACEDIR] is \"/home/\$SCRIPTUSER/catkin_ws\")"
echo "WORKSPACEDIR must specify the absolute path of the directory"
echo "-f sets FORCE=-f and will force a (re)install of all compiled-from-source components."

# find O/S codename (set to UCODENAME)
source ./get_os_codename.sh

#
# find path of this-script-being-run
# see: http://stackoverflow.com/questions/630372/determine-the-path-of-the-executing-bash-script
#
RELATIVE_PATH="`dirname \"$0\"`"
ABSOLUTE_PATH="`( cd \"$RELATIVE_PATH\" && pwd )`"
echo "PATH of current script ($0) is: $ABSOLUTE_PATH"

#
# INPUT ARGUMENT PARSING:
#

# set defaults for input arguments
ROSVERSION=
SCRIPTUSER=vagrant
WORKSPACEDIR="/home/$SCRIPTUSER/catkin_ws"
FORCE=









#
# NOTE: this file does the following:
# (does NOT install ROS indigo, as ROS indigo is pre-installed on the "shadowrobot/ros-indigo-desktop-trusty64" base box)
# (if requested, installs ROS jade on "ubuntu/trusty64" (add servers to apt-get list, add key, then install))
# (if requested, installs ROS kinetic on "ubuntu/xenial64" (add servers to apt-get list, add key, then install))
#

#
# run installation + upgrades
#

# update all packages, because "gah!" otherwise, especially for 'rosdep' stuff later
sudo apt-get -y update
sudo apt-get -y upgrade

sudo apt-get -y install wget curl # for wget and possible curl use below

#
# install ROS indigo OR jade OR kinetic (for "ubuntu/trusty64" box)
# --> can comment out for "shadowrobot/ros-indigo-desktop-trusty64" box (ROS indigo is pre-installed on that Vagrantbox)
#
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net:80 --recv-key 0xB01FA116
sudo apt-get -y update
sudo apt-get -y install ros-$ROSVERSION-desktop-full # will not hurt anything if preinstalled
# if rosdep sources file list has -not- already been initialized:
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then # rosdep init this
    sudo rosdep init
    su - $SCRIPTUSER -c "rosdep update;"
fi
sudo apt-get -y install python-rosinstall






echo "End of install_appropriate_ros_version.sh script!"