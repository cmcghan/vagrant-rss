#!/bin/bash -e
# Copyright by California Institute of Technology, University of Cincinnati
# All rights reserved. See LICENSE file at:
# https://github.com/cmcghan/vagrant-rss

#
# in the initialization process for vagrant, this bash script is run as user 'root' from /vagrant
#

echo "Start of install_p3dx_ros.sh script!"
#echo "input arguments: ROSVERSION [SCRIPTUSER] [WORKSPACEDIR] [-f]"

#
# find path of this-script-being-run
# see: http://stackoverflow.com/questions/630372/determine-the-path-of-the-executing-bash-script
#
RELATIVE_PATH="`dirname \"$0\"`"
ABSOLUTE_PATH="`( cd \"$RELATIVE_PATH\" && pwd )`"
echo "PATH of current script ($0) is: $ABSOLUTE_PATH"

#
# parse input vars (set to appropriate vars or default vars)
#
source $ABSOLUTE_PATH/get_rv_su_wd_f.sh "$@"
# when source'd, sets these vars at this level: ROSVERSION SCRIPTUSER WORKSPACEDIR FORCE


#
# run installation + upgrades
#

# update all packages, because "gah!" otherwise, especially for 'rosdep' stuff later
$ABSOLUTE_PATH/apt_upd_sys.sh

sudo apt-get -y install wget curl # for wget and possible curl use below

sudo -u $SCRIPTUSER mkdir -p $WORKSPACEDIR/src

# install (SD-Robot-Vision / ua_ros_p3dx) libraries for ./rss_git/contrib/p3dx_gazebo_mod
sudo apt-get -y install ros-$ROSVERSION-controller-manager-tests
sudo apt-get -y install ros-$ROSVERSION-ros-controllers
if [ "$ROSVERSION" == "indigo" ]; then
    sudo apt-get -y install ros-$ROSVERSION-gazebo-ros-control
elif [ "$ROSVERSION" == "jade" ]; then
    sudo apt-get -y install ros-$ROSVERSION-gazebo-ros-pkgs # does not include ros-jade-gazebo-ros-control yet... do we need it? if we do, then:
    echo "ROS $ROSVERSION doesn't have ros-$ROSVERSION-ros-control package in ros-$ROSVERSION-gazebo-ros-pkgs (yet). Source install via 'git clone' now:"
    cd $WORKSPACEDIR/src    
    if [ "$FORCE" == "-f" ]; then
        rm -rf gazebo_ros_pkgs
    fi
    sudo -s $SCRIPTUSER git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git # includes gazebo_ros_control...
    sudo apt-get -y install ros-$ROSVERSION-ros-control # for gazebo_ros_control, need transmission_interface
elif [ "$ROSVERSION" == "kinetic" ]; then
    sudo apt-get -y install ros-$ROSVERSION-gazebo-ros-control
fi
# then install the p3dx gazebo model from github
cd $WORKSPACEDIR/src
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

#now, catkin_make this bad boy! :)
su - $SCRIPTUSER -c "source /home/$SCRIPTUSER/.bashrc; cd $WORKSPACEDIR; source /opt/ros/$ROSVERSION/setup.bash; /opt/ros/$ROSVERSION/bin/catkin_make;"

echo "End of install_p3dx_ros.sh script!"
