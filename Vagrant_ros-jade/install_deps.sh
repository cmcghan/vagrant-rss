#!/bin/bash -e

#
# in the initialization process for vagrant, this bash script is run as user 'root' from /vagrant
#

#
# Note: Ubuntu X-Windows Desktop and ROS indigo are pre-installed
# on the "shadowrobot/ros-indigo-desktop-trusty64" base box
#

echo "Start of install_deps.sh script!"

#
# get -f (force) if given
#

# if we get an input parameter (username) then use it, else use default 'vagrant'
if [ $# -eq 1 ] && [ "$1" == "-f" ]
then
    echo "-f (force) commandline argument given. Forcing install of all compiled-from-source components."
    echo "Note: will -NOT- attempt to force-reinstall RosAria!"
    FORCE=$1
else
    FORCE=
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

# start in the /root directory
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
sudo -u vagrant mkdir -p /home/vagrant/catkin_ws/src

# just in case, fix ownership of /home/vagrant/catkin_ws/src
sudo chown -R vagrant:vagrant /home/vagrant/catkin_ws

# install gnome-terminal for multiscript*.py runs
# install rosbridge
# install turtlebot libraries
# install (SD-Robot-Vision / ua_ros_p3dx) libraries for ./rss_git/contrib/p3dx_gazebo_mod
# set up catkin workspace
# install ROSARIA
# install deps for MobileSim and MobileSim
# install python WebSocket library
/vagrant/single_installers/install_rosstuff_setup_catkinworkspace.sh vagrant $FORCE

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

echo "End of install_deps.sh script!"