#!/bin/bash -e
# Copyright by California Institute of Technology, University of Cincinnati
# All rights reserved. See LICENSE file at:
# https://github.com/cmcghan/vagrant-rss

#
# in the initialization process for vagrant, this bash script is run as user 'root' from /vagrant
#

echo "Start of install_gazebo_plus_rospkgs.sh script!"
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

# install gazebo and gazebo-ros packages
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
mkdir -p initdeps
cd ~/initdeps
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get -y update
if [ "$ROSVERSION" -eq "indigo" ]; then # install gazebo2, too
    sudo apt-get -y install gazebo2 gazebo2-dbg ros-$ROSVERSION-gazebo-ros
elif [ "$ROSVERSION" -eq "jade" ]; then # install gazebo5, too
    sudo apt-get -y install gazebo5 libgazebo5-dev
    # set up gazebo 5.1 via:
    sudo cp -r /usr/share/gazebo-5.0/media /usr/share/gazebo-5.1/
    sudo cp -r /usr/share/gazebo-5.0/worlds /usr/share/gazebo-5.1/
    # then source the gazebo directories and run gazebo...
    #cd /usr/share/gazebo-5.1 ? cd /usr/share/gazebo-5.0 ?
    #source /usr/share/gazebo-5.1/setup.sh 
    #gazebo --verbose
    # ...and then grab all the models from online that you want!
elif [ "$ROSVERSION" -eq "kinetic" ]; then # install gazebo7, too
    sudo apt-get -y install gazebo7 libgazebo7-dev
fi


echo "End of install_gazebo_plus_rospkgs.sh script!"