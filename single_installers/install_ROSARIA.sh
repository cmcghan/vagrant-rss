#!/bin/bash -e
# Copyright by California Institute of Technology, University of Cincinnati
# All rights reserved. See LICENSE file at:
# https://github.com/cmcghan/vagrant-rss

#
# in the initialization process for vagrant, this bash script is run as user 'root' from /vagrant
#

echo "Start of install_ROSARIA.sh script!"

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

# install ROSARIA (reference: http://wiki.ros.org/ROSARIA/Tutorials/How%20to%20use%20ROSARIA )
cd $WORKSPACEDIR/src
# do NOT attempt to force rosaria if it's been installed previously! could cause serious issues (esp. with Aria and libaria!)
# (Dependencies: genmsg -  if the make fails, Install package separately from here: https://github.com/ros/genmsg)
if [ ! -d rosaria ]
then
    sudo -u $SCRIPTUSER git clone https://github.com/amor-ros-pkg/rosaria.git
    cd rosaria
    # roll back to earlier version (/cmd_vel as pub-sub)
    sudo -u $SCRIPTUSER git checkout d58a244fc50bba568da57a828c9121c83a19284d
    # this checkout downgrades to a version of rosaria that does not
    # (a) require: chmod +x cfg/RosAria.cfg
    # (b) require: pubtopic change from /RosAria/cmd_vel to /cmd_vel
    su - $SCRIPTUSER -c "source /home/$SCRIPTUSER/.bashrc; cd $WORKSPACEDIR; source devel/setup.bash; rosdep update; rosdep -y install rosaria; /opt/ros/$ROSVERSION/bin/catkin_make;"
    # must run "rosdep update" and "rosdep install rosaria" as $SCRIPTUSER (non-root user)
    # sudo rosdep fix-permissions # if was root, would then need to run 'rosdep update' again without sudo
fi




echo "End of install_ROSARIA.sh script!"