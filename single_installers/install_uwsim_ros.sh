#!/bin/bash -e
# Copyright by California Institute of Technology, University of Cincinnati
# All rights reserved. See LICENSE file at:
# https://github.com/cmcghan/vagrant-rss

#
# in the initialization process for vagrant, this bash script is run as user 'root' from /vagrant
#

echo "Start of install_uwsim_ros.sh script!"
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

# install UWSim stuff
cd /home/$SCRIPTUSER/catkin_ws/src
sudo apt-get -y install python-urlgrabber ros-indigo-uwsim libdc1394-22 libdc1394-22-dev libdc1394-utils
if [ "$FORCE" == "-f" ]; then
    rm -rf freefloating_gazebo
    rm -rf freefloating_gazebo_demo
fi
if [ ! -d freefloating_gazebo ]; then
    sudo -u $SCRIPTUSER git clone https://github.com/freefloating-gazebo/freefloating_gazebo
fi
if [ ! -d freefloating_gazebo_demo ]; then
    sudo -u $SCRIPTUSER git clone https://github.com/freefloating-gazebo/freefloating_gazebo_demo
fi
su - $SCRIPTUSER -c "source /home/$SCRIPTUSER/.bashrc; cd /home/$SCRIPTUSER/catkin_ws; source /opt/ros/$ROSVERSION/setup.bash; /opt/ros/$ROSVERSION/bin/catkin_make;"
# set up demo for the first time:
#0$ cd ~/catkin_ws && source devel/setup.bash && roscore # is this necessary?
#1$ cd ~/catkin_ws && source devel/setup.bash
#1$ rosrun uwsim uwsim # to download robots (~/.uwsim/data)
#1$ roslaunch freefloating_gazebo_demo g500arm5e.launch parse:=true # synchs simulators (creates urdf from xacro for UWSim)
# start the simulators:
#1$ cd ~/catkin_ws && source devel/setup.bash
#1$ roslaunch freefloating_gazebo_demo g500arm5e.launch
#2$ cd ~/catkin_ws && source devel/setup.bash
#2$ roslaunch freefloating_gazebo_demo g500arm5e_gazebo.launch uwsim:=true # set uwsim:=false to show window
# test the demo:
#3$ cd ~/catkin_ws && source devel/setup.bash
#3$ rosrun freefloating_gazebo_demo freefloating_gazebo_demo
# alt. is to test the demo via manual teleoperation:
#3$ cd ~/catkin_ws && source devel/setup.bash
#3$ roslaunch freefloating_gazebo_demo manual.launch




echo "End of install_uwsim_ros.sh script!"