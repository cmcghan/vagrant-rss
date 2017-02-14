#!/bin/bash -e
# Copyright by California Institute of Technology, University of Cincinnati
# All rights reserved. See LICENSE file at:
# https://github.com/cmcghan/vagrant-rss

#
# in the initialization process for vagrant, this bash script is run as user 'root' from /vagrant
#

echo "Start of install_ws4py.sh script!"
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
# check for installation
#

WS4PY_FOUND=`python -c "import pkg_resources; print(pkg_resources.get_distribution('ws4py').version)" | grep -m 1 -o "0.3.5" | wc -l`
# pkg_resources should give '0.3.5'
# grep should find a match and repeat it
# and wc -l should give 1 if ws4py 0.3.5 was found
if [ $WS4PY_FOUND -eq 1 ]; then
    echo "ws4py 0.3.5 already installed!"
fi
# longer test from within python:
# >>> from ws4py.client.threadedclient import WebSocketClient
# >>> testclient = WebSocketClient('ws://localhost:9090/')

#
# run installation + upgrades
#

# update all packages, because "gah!" otherwise, especially for 'rosdep' stuff later
sudo apt-get -y update
sudo apt-get -y upgrade

sudo apt-get -y install wget curl # for wget and possible curl use below

# install python WebSocket library (reference: https://ws4py.readthedocs.org/en/latest/sources/install/ )
mkdir -p ~/initdeps/rosbridgeclient
cd ~/initdeps/rosbridgeclient
# if need to force, then remove old directory first
if [ "$FORCE" == "-f" ]; then
    rm -rf "WebSocket-for-Python"
fi
if [ ! -d "WebSocket-for-Python" ]; then
    git clone "https://github.com/Lawouach/WebSocket-for-Python.git"
fi
cd "WebSocket-for-Python"
if [ "$FORCE" == "-f" ] || [ $WS4PY_FOUND -eq 0 ]; then
    sudo python setup.py install
fi



echo "End of install_ws4py.sh script!"