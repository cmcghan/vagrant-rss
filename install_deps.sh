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
echo "input arguments: ROSVERSION [SCRIPTUSER] [FORCE (-f)]"
echo "(note: order of [SCRIPTUSER] and -f argument can be swapped)"
echo "(note: default SCRIPTUSER is \"vagrant\")"

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
#/vagrant/single_installers/install_tulip1.1a.sh $FORCE
/vagrant/single_installers/install_tulip1.2.0.sh $FORCE

#
# install other RSE dependencies:
#

# install recommended software for python development (python-pip already installed above)
sudo apt-get -y install spyder geany python-dev build-essential dos2unix

# install polytope library (if not already installed via tulip-1.2.0) (python-pip already installed above)
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

# OMPL install moved to bottom of file due to possible installation issues under some circumstances

# install Matlab toolboxes for deliberative/psulu-jpl-matlab
# --> to be added! (lpsolve or gurobi or cplex, yalmip, Matlab-Ros-Interface)
# see: https://docs.google.com/document/d/1VlQE635KTaDyKJeF1kbbQNOqq5Df3gCpgYcoYGBTKTg/edit#heading=h.7efxum6agk2e

# install python libraries for deliberative/pSulu-jpl-python:
sudo apt-get -y install python-mpmath python-pip
sudo pip install pulp

# install python libraries for deliberative/psulu_picard (doxygen installed above)
/vagrant/single_installers/install_ipopt.sh $FORCE

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
/vagrant/single_installers/install_ompl.sh $FORCE

# install Google TensorFlow for Ravi Kiran's machine learning work:
/vagrant/single_installers/install_tensorflow0.8.0.sh $SCRIPTUSER $FORCE
# note that this installs TensowFlow to a VirtualEnv session for the given $SCRIPTUSER
# also note that the python3 install may not work / may error out...

echo "End of install_deps.sh script!"
