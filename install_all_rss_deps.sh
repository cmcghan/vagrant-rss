#!/bin/bash -e
# Copyright by California Institute of Technology, University of Cincinnati
# All rights reserved. See LICENSE file at:
# https://github.com/cmcghan/vagrant-rss

#
# in the initialization process for vagrant, this bash script is run as user 'root' from /vagrant
#

#
# Note: Ubuntu X-Windows Desktop and ROS indigo are pre-installed
# on the "shadowrobot/ros-indigo-desktop-trusty64" base box
# but -not- on either of the "ubuntu/trusty64" or "ubuntu/xenial64" base boxes
#

echo "Start of install_deps.sh script!"
#echo "input arguments: ROSVERSION [SCRIPTUSER] [WORKSPACEDIR] [-f]"

#
# find path of this-script-being-run
# see: http://stackoverflow.com/questions/630372/determine-the-path-of-the-executing-bash-script
#
RELATIVE_PATH="`dirname \"$0\"`"
ABSOLUTE_PATH="`( cd \"$RELATIVE_PATH\" && pwd )`"
echo "PATH of current script ($0) is: $ABSOLUTE_PATH"

# find O/S codename (set to UCODENAME)
source $ABSOLUTE_PATH/single_installers/get_os_codename.sh

#
# parse input vars (set to appropriate vars or default vars)
#
source $ABSOLUTE_PATH/get_rv_su_wd_f.sh "$@"
# when source'd, sets these vars at this level: ROSVERSION SCRIPTUSER WORKSPACEDIR FORCE

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
#/vagrant/single_installers/install_glpk_cvxopt.sh $FORCE
$ABSOLUTE_PATH/single_installers/install_glpk_cvxopt.sh $FORCE
    
# install gr1c:
#/vagrant/single_installers/install_gr1c.sh $FORCE
$ABSOLUTE_PATH/single_installers/install_gr1c.sh $FORCE

# install tulip-control v1.1a system-wide
#/vagrant/single_installers/install_tulip1.1a.sh $FORCE
#$ABSOLUTE_PATH/vagrant/single_installers/install_tulip1.1a.sh $FORCE
#/vagrant/single_installers/install_tulip1.2.0.sh $FORCE
$ABSOLUTE_PATH/single_installers/install_tulip1.2.0.sh $FORCE

#
# install other RSE dependencies:
#

# install recommended software for python development (python-pip already installed above)
sudo apt-get -y install spyder geany python-dev build-essential dos2unix

# install polytope library (if not already installed via tulip installer) (python-pip already installed above)
#sudo apt-get -y install python-numpy python-scipy python-cvxopt python-networkx python-pip
##sudo pip install polytope # won't force an upgrade if not installed before # current version in repo (0.1.1) has issues with numpy >= 1.10 (because of version string parsing in quickhull.py); note that polytope won't work properly with numpy <=1.5.9 (unique1d() instead of unique())
#sudo pip install --upgrade pip
#sudo pip install --upgrade numpy
#sudo pip install --upgrade scipy
#cd ~/initdeps
#if [ ! -d polytope ]
#then
#    git clone https://github.com/tulip-control/polytope.git
#    cd polytope
#    sudo pip install . # pip install polytope from local download
#fi
#if [ "$FORCE" == "-f" ]
#then
#    sudo pip install --upgrade polytope # do this to force newest version of polytope (and other deps: numpy, scipy, cvxopt, networkx) to install (polytope 0.1.1 as of 2016-04-20)
#fi

cd ~/initdeps

# directory should exist, but just to make sure...
sudo -u $SCRIPTUSER mkdir -p $WORKSPACEDIR/src

# just in case, fix ownership of /home/$SCRIPTUSER/catkin_ws/src
sudo chown -R $SCRIPTUSER:$SCRIPTUSER $WORKSPACEDIR

# install gnome-terminal for multiscript*.py runs
# install rosbridge
# install turtlebot libraries
# install (SD-Robot-Vision / ua_ros_p3dx) libraries for ./rss_git/contrib/p3dx_gazebo_mod
# set up catkin workspace
# install ROSARIA
# install deps for MobileSim and MobileSim
# install python WebSocket library
#/vagrant/single_installers/install_rosstuff_setup_catkinworkspace.sh $ROSVERSION $SCRIPTUSER $FORCE
$ABSOLUTE_PATH/single_installers/install_rosstuff_setup_catkinworkspace.sh $ROSVERSION $SCRIPTUSER $WORKSPACEDIR $FORCE

# OMPL install moved to bottom of file due to possible installation issues under some circumstances

# install Matlab toolboxes for deliberative/psulu-jpl-matlab
# --> to be added! (lpsolve or gurobi or cplex, yalmip, Matlab-Ros-Interface)
# see: https://docs.google.com/document/d/1VlQE635KTaDyKJeF1kbbQNOqq5Df3gCpgYcoYGBTKTg/edit#heading=h.7efxum6agk2e

# install python libraries for deliberative/pSulu-jpl-python:
sudo apt-get -y install python-mpmath python-pip
sudo pip install pulp
if [ "$FORCE" == "-f" ]
then
    sudo pip install --upgrade pulp # do this to force newest version (and other deps) to install
fi

# install python libraries for deliberative/psulu_picard (doxygen installed above)
#/vagrant/single_installers/install_ipopt.sh $FORCE
$ABSOLUTE_PATH/single_installers/install_ipopt.sh $FORCE

# install python libraries for Michele Colledanchise's behavioral tree stuff:
sudo apt-get -y install libgeos-dev # Geometry Engine Open Source (GEOS) needed for shapely
sudo pip install shapely
# now, Michele Colledanchise's Behavior Tree work :)
cd ~/initdeps
if [ "$FORCE" == "-f" ]
then
    rm -rf behavior_tree
fi
if [ ! -d behavior_tree ]; then
    git clone https://github.com/miccol/Behavior-Tree.git behavior_tree
fi
cd behavior_tree
mkdir ./build
cd build
cmake ..
make # "Note the installation generates the shared library in behavior_tree/build/lib and the sample code in behavior_tree/build/sample"
sudo make install # to install system-wide
# check installation via:
#cd ~/initdeps/behavior_tree
#cd build/sample
#./btpp_example
# or:
# /usr/local/bin/btpp_example

# install eclipse for development environment for Oktay's RRT# (RRT-sharp) planner
sudo apt-get -y install eclipse-platform
# install dependencies for Ravi's ROS-native wrapper for the RRT# code (service call)
sudo apt-get -y install gcc g++ patch wget
# C++ Armadillo library: http://arma.sourceforge.net/download.html
sudo apt-get -y install cmake libopenblas-dev liblapack-dev libatlas-dev libarpack2-dev libsuperlu3-dev # libsuperlu-dev # libarpack-dev
sudo apt-get -y install libarmadillo-dev # installs 4.200.0, not 6.600.5 latest
cd ~/initdeps
wget http://sourceforge.net/projects/arma/files/armadillo-6.600.5.tar.gz
tar xvzf armadillo-6.600.5.tar.gz
cd armadillo-6.600.5
cmake .
make
sudo make install
# YAML-CPP: https://github.com/jbeder/yaml-cpp/releases/tag/release-0.5.3
# This is the patched release. Latest git pull has an issue ( https://github.com/jbeder/yaml-cpp/commit/34bd1a7083e5875e6a4b2d4f61c0b356cc5d53fc )
cd ~/initdeps
wget https://github.com/jbeder/yaml-cpp/archive/release-0.5.3.tar.gz
tar xvzf release-0.5.3.tar.gz
cd yaml-cpp-release-0.5.3
mkdir build
cd build
cmake .. -DBUILD_SHARED_LIBS=ON
make clean all
make
sudo make install
# then take rrtsharp.tar.gz, place in ~/catkin_ws/src, and catkin_make inside ~/catkin_ws
#pull_from_wherever
#cd /home/$SCRIPTUSER/
#sudo -u vagrant tar xvzf rrtsharp.tar.gz # extracts to directory /home/$SCRIPTUSER/catkin_ws/src/rrt
#cd ..
#su - $SCRIPTUSER -c "source /home/$SCRIPTUSER/.bashrc; cd /home/$SCRIPTUSER/catkin_ws; source devel/setup.bash; /opt/ros/$ROSVERSION/bin/catkin_make;"
# check that functions like "driver" are available as dynamically-linked functions in the library with static names
#python
#import ctypes
#ctypes.CDLL
##<class 'ctypes.CDLL'>
#ctypes.CDLL("librrt.so")
##<CDLL 'librrt.so', handle 2257580 at 7f5b71fde110>
#ctypes.CDLL("librrt.so").driver
##<_FuncPtr object at 0x7f5b71f7a460>
#exit()

# install yaml in Python (PyYAML with LibYAML bindings) for yaml read-ins
# Instructions here: 
# * http://pyyaml.org/wiki/PyYAMLDocumentation
# * http://pyyaml.org/wiki/PyYAML
# * http://pyyaml.org/wiki/LibYAML
#
# install PyYAML via:
# $ wget http://pyyaml.org/download/pyyaml/PyYAML-3.11.tar.gz
# $ tar xvzf PyYAML-3.11.tar.gz
# $ cd PyYAML-3.11
# $ sudo python setup.py install
#
# for LibYAML bindings instead:
# install LibYAML via:
cd ~/initdeps
wget http://pyyaml.org/download/libyaml/yaml-0.1.5.tar.gz
tar xvzf yaml-0.1.5.tar.gz
cd yaml-0.1.5
./configure
make
sudo make install
# download PyYAML above, then install using LibYAML bindings:
cd ~/initdeps
wget http://pyyaml.org/download/pyyaml/PyYAML-3.11.tar.gz
tar xvzf PyYAML-3.11.tar.gz
cd PyYAML-3.11
sudo python setup.py --with-libyaml install
#
# then:
#
#from yaml import load, dump
#try: # LibYAML
#    from yaml import CLoader as Loader, CDumper as Dumper
#except ImportError: # fallback on PyYAML
#    from yaml import Loader, Dumper
# ...
#data = load(stream, Loader=Loader)
# ...
#output = dump(data, Dumper=Dumper)

# install OMPL libraries (cvxopt and glpk already installed above)
#/vagrant/single_installers/install_ompl.sh $FORCE
$ABSOLUTE_PATH/single_installers/install_ompl.sh $FORCE

# install Google TensorFlow for Ravi Kiran's machine learning work:
#/vagrant/single_installers/install_tensorflow0.8.0.sh $SCRIPTUSER $FORCE
$ABSOLUTE_PATH/single_installers/install_tensorflow0.8.0.sh $SCRIPTUSER $FORCE
# note that this installs TensowFlow to a VirtualEnv session for the given $SCRIPTUSER
# also note that the python3 install may not work / may error out...

echo "End of install_deps.sh script!"
