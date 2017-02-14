#!/bin/bash -e
# Copyright by California Institute of Technology, University of Cincinnati
# All rights reserved. See LICENSE file at:
# https://github.com/cmcghan/vagrant-rss

#
# in the initialization process for vagrant, this bash script is run as user 'root' from /vagrant
#

echo "Start of install_p3dx_ros.sh script!"
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
# run installation + upgrades
#

# update all packages, because "gah!" otherwise, especially for 'rosdep' stuff later
sudo apt-get -y update
sudo apt-get -y upgrade

sudo apt-get -y install wget curl # for wget and possible curl use below

cd /home/$SCRIPTUSER
sudo -u $SCRIPTUSER mkdir -p catkin_ws/src

# install (SD-Robot-Vision / ua_ros_p3dx) libraries for ./rss_git/contrib/p3dx_gazebo_mod
sudo apt-get -y install ros-$ROSVERSION-controller-manager-tests
sudo apt-get -y install ros-$ROSVERSION-ros-controllers
if [ "$ROSVERSION" -eq "indigo" ]; then
    sudo apt-get -y install ros-$ROSVERSION-gazebo-ros-control
elif [ "$ROSVERSION" -eq "jade" ]; then
    sudo apt-get -y install ros-$ROSVERSION-gazebo-ros-pkgs # does not include ros-jade-gazebo-ros-control yet... do we need it? if we do, then:
    echo "ROS $ROSVERSION doesn't have ros-$ROSVERSION-ros-control package in ros-$ROSVERSION-gazebo-ros-pkgs (yet). Source install via 'git clone' now:"
    cd /home/$SCRIPTUSER/catkin_ws/src    
    if [ "$FORCE" == "-f" ]; then
        rm -rf gazebo_ros_pkgs
    fi
    sudo -s $SCRIPTUSER git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git # includes gazebo_ros_control...
    sudo apt-get -y install ros-jade-ros-control # for gazebo_ros_control, need transmission_interface
elif [ "$ROSVERSION" -eq "kinetic" ]; then
    sudo apt-get -y install ros-$ROSVERSION-gazebo-ros-control
fi
# then install the p3dx gazebo model from github
cd /home/$SCRIPTUSER/catkin_ws/src
# if need to force, then remove old directory first
if [ "$FORCE" == "-f" ]; then
    rm -rf PioneerModel
fi
if [ ! -d PioneerModel ]; then
    sudo -u $SCRIPTUSER git clone https://github.com/SD-Robot-Vision/PioneerModel.git
fi
# if need to force, then remove old directory first
if [ "$FORCE" == "-f" ]; then
    rm -rf p3dx_mover
fi
if [ ! -d p3dx_mover ]; then
    sudo -u $SCRIPTUSER git clone https://github.com/SD-Robot-Vision/p3dx_mover.git
fi
# PioneerModel/p3dx_control requires controller_manager to compile








echo "End of install_p3dx_ros.sh script!"