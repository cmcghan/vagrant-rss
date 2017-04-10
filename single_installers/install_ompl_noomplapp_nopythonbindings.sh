#!/bin/bash -e
# Copyright by California Institute of Technology, University of Cincinnati
# All rights reserved. See LICENSE file at:
# https://github.com/cmcghan/vagrant-rss

#
# AS OF 2017-01-18 (or earlier):
#

# note that there is now an installation script provided by Kavrakilab for Ubuntu, Fedora, Linux (generic), OS X, and Windows(!!)
# it can be downloaded and permissions set to run via:
#wget http://ompl.kavrakilab.org/install-ompl-ubuntu.sh
#chmod u+x install-ompl-ubuntu.sh

# As per ( http://ompl.kavrakilab.org/installation.html ) :
# "There are three ways to run the script:
#    ./install-ompl-ubuntu.sh will install OMPL without Python bindings
#    ./install-ompl-ubuntu.sh --python will install OMPL with Python bindings
#    ./install-ompl-ubuntu.sh --app will install OMPL.app with Python bindings
# The script downloads and installs OMPL and all dependencies via apt-get & pip and from source.
# It will ask for your password to install things. The script has been tested on vanilla installs
# of Ubuntu 14.04 (Trusty), 15.10 (Wily), and 16.04 (Xenial)."

# Alternately, if the source code and/or Python bindings and/or GUI/app aren't needed...
sudo apt-get -y install libompl-dev ompl-demos

# Alternately-alternately, if you want the ROS MoveIt! version and have already installed a version of ROS...
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get -y install ros-`rosversion -d`-ompl

# --EOF--
