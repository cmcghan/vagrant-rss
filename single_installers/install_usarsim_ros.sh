#!/bin/bash -e
# Copyright by California Institute of Technology, University of Cincinnati
# All rights reserved. See LICENSE file at:
# https://github.com/cmcghan/vagrant-rss

#
# in the initialization process for vagrant, this bash script is run as user 'root' from /vagrant
#

echo "Start of install_usarsim_ros.sh script!"

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
# parse input vars (set to appropriate vars or default vars)
#
source $ABSOLUTE_PATH/get_rv_su_wd_f.sh "$@"
# when source'd, sets these vars at this level: ROSVERSION SCRIPTUSER WORSPACEDIR FORCE


#
# run installation + upgrades
#

# update all packages, because "gah!" otherwise, especially for 'rosdep' stuff later
sudo apt-get -y update
sudo apt-get -y upgrade

sudo apt-get -y install wget curl # for wget and possible curl use below

# install USARSimROS + libraries
#sudo apt-get -y install ???
#cd /home/$SCRIPTUSER/initdeps/
#wget http://downloads.sourceforge.net/project/usarsim/usarsim-UDK/USARSimFull_UDKV1.2.zip
#unzip 
#cd /home/$SCRIPTUSER/catkin_ws/src
## if need to force, then remove old directory first
#if [ "$FORCE" == "-f" ]; then
#    rm -rf PioneerModel
#fi
#if [ ! -d PioneerModel ]; then
#    sudo -u $SCRIPTUSER git clone https://github.com/SD-Robot-Vision/PioneerModel.git
#fi

if [ "$ROSVERSION" -eq "indigo" ]; then
    
elif [ "$ROSVERSION" -eq "jade" ]; then
    
elif [ "$ROSVERSION" -eq "kinetic" ]; then
    
fi

#now, catkin_make this bad boy! :)
su - $SCRIPTUSER -c "source /home/$SCRIPTUSER/.bashrc; cd $WORKSPACEDIR; /opt/ros/$ROSVERSION/bin/catkin_make;"


echo "End of install_usarsim_ros.sh script!"