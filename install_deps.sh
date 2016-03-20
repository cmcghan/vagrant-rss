#!/bin/bash -e
# Copyright by California Institute of Technology
# All rights reserved. See LICENSE file at:
# https://github.com/cmcghan/vagrant-rss

#
# in the initialization process for vagrant, this bash script is run as user 'root' from /vagrant
#

#
# Note: Ubuntu X-Windows Desktop and ROS indigo are pre-installed
# on the "shadowrobot/ros-indigo-desktop-trusty64" base box
#

echo "Start of install_deps.sh script!"
echo "input arguments: ROSVERSION [SCRIPTUSER] [FORCE]"
echo "(note: order of [SCRIPTUSER] and [FORCE] can be swapped)"

# set defaults for input arguments
ROSVERSION=
SCRIPTUSER="vagrant"
FORCE=
# if we get an input parameter (username) then use it, else use default 'vagrant'
# get -f (force) if given -- NOTE: WILL -NOT- REMOVE OR FORCE-REINSTALL ROSARIA!!!
if [ $# -lt 1 ]; then
    echo "ERROR: No ROS version given as commandline argument. Exiting."
    exit
else # at least 1 (possibly 3) argument(s) at commandline...
    if [ $1 == "indigo" ]; then
        ROSVERSION="indigo"
    elif [ $1 == "jade" ]; then
        ROSVERSION="jade"
    else
        echo "ERROR: Unknown ROS version given as commandline argument. Exiting."
        exit
    fi
    echo "ROS version is $ROSVERSION."
    if [ $# -gt 1 ]; then # at least 2 (possibly more) arguments at commandline...
        if [ "$2" == "-f" ]; then
            echo "-f (force) commandline argument given."
            FORCE=$2
        else
            echo "Username given as commandline argument."
            SCRIPTUSER=$2
        fi
        if [ $# -gt 2 ]; then # at least 3 (possibly more) arguments at commandline...
            if [ "$3" == "-f" ]; then
                echo "-f (force) commandline argument given."
                FORCE=$3
            elif [ $SCRIPTUSER -eq "vagrant" ]; then
                echo "Username given as commandline argument."
                SCRIPTUSER=$3
            else
                echo "Username already set. Second argument ignored."
            fi
        fi
    fi
fi
echo "Will be using user $SCRIPTUSER and directories at and under /home/$SCRIPTUSER..."
if [ $FORCE -eq "-f" ]; then
    echo "Forcing install of all compiled-from-source components."
fi

#
# run installation + upgrades
#

# update all packages, because "gah!" otherwise, especially for 'rosdep' stuff later
sudo apt-get -y update
sudo apt-get -y upgrade

#
# install dependencies for tulip-control:
#

# start in the root directory (if "sudo su" then is "/root")
cd ~
# make and move into directory for holding compilation files + downloads
mkdir -p initdeps
cd initdeps

# back to compilation/install directory (/root/initdeps)
#cd ~/initdeps

# install glpk and cvxopt:
/vagrant/single_installers/install_glpk_cvxopt.sh $FORCE
    
# install gr1c:
/vagrant/single_installers/install_gr1c.sh $FORCE

# install tulip-control v1.1a system-wide
/vagrant/single_installers/install_tulip1.1a.sh $FORCE

#
# install other RSE dependencies:
#

# install recommended software for python development (python-pip already installed above)
sudo apt-get -y install spyder geany python-dev build-essential dos2unix

# install polytope library (python-pip already installed above)
sudo apt-get -y install python-numpy python-scipy python-cvxopt python-networkx python-pip
sudo pip install polytope

# directory should exist, but just to make sure...
sudo -u $SCRIPTUSER mkdir -p /home/$SCRIPTUSER/catkin_ws/src

# just in case, fix ownership of /home/$SCRIPTUSER/catkin_ws/src
sudo chown -R $SCRIPTUSER:$SCRIPTUSER /home/$SCRIPTUSER/catkin_ws

# install gnome-terminal for multiscript*.py runs
# install rosbridge
# install turtlebot libraries
# install (SD-Robot-Vision / ua_ros_p3dx) libraries for ./rss_git/contrib/p3dx_gazebo_mod
# set up catkin workspace
# install ROSARIA
# install deps for MobileSim and MobileSim
# install python WebSocket library
/vagrant/single_installers/install_rosstuff_setup_catkinworkspace.sh $ROSVERSION $SCRIPTUSER $FORCE

# install OMPL libraries (cvxopt and glpk already installed above)
/vagrant/single_installers/install_ompl.sh $FORCE

# install Matlab toolboxes for deliberative/psulu-jpl-matlab
# --> to be added! (lpsolve or gurobi or cplex, yalmip, Matlab-Ros-Interface)
# see: https://docs.google.com/document/d/1VlQE635KTaDyKJeF1kbbQNOqq5Df3gCpgYcoYGBTKTg/edit#heading=h.7efxum6agk2e

# install python libraries for deliberative/pSulu-jpl-python:
sudo apt-get -y install python-mpmath python-pip
sudo pip install pulp

# install python libraries for deliberative/psulu_picard (doxygen installed above)
/vagrant/single_installers/install_ipopt.sh $FORCE

# install python libraries for Michele Colledanchise's behavioral tree stuff:
sudo apt-get install libgeos-dev # Geometry Engine Open Source (GEOS) needed for shapely
sudo pip install shapely

echo "End of install_deps.sh script!"
