#!/bin/bash -e
# Copyright by California Institute of Technology
# All rights reserved. See LICENSE file at:
# https://github.com/cmcghan/vagrant-rss

#
# in the initialization process for vagrant, this bash script is run as user 'root' from /vagrant
#

echo "Start of install_rosstuff_setup_catkinworkspace.sh script!"

#
# NOTE: this file does the following:
# install ROS jade (add servers to apt-get list, add key, then install)
# install gnome-terminal for multiscript*.py runs
# install rosbridge
# install turtlebot libraries (-=currently limited!!!=-)
# install (SD-Robot-Vision / ua_ros_p3dx) libraries for ./rss_git/contrib/p3dx_gazebo_mod
# set up catkin workspace
# install ROSARIA
# install deps for MobileSim and MobileSim
# install python WebSocket library
#

#
# get user directory
#

# if we get an input parameter (username) then use it, else use default 'vagrant'
if [ $# -lt 1 ]
then
    echo "Single username not given as commandline argument. Using default."
    SCRIPTUSER="vagrant"
else
    echo "Username given as commandline argument."
    SCRIPTUSER=$1
fi
echo "Will be using user $SCRIPTUSER and directories at and under /home/$SCRIPTUSER..."

#
# get -f (force) if given -- NOTE: WILL -NOT- REMOVE OR FORCE-REINSTALL ROSARIA!!!
#

# if we get an input parameter (username) then use it, else use default 'vagrant'
if [ $# -eq 2 ] && [ "$2" == "-f" ]
then
    echo "-f (force) commandline argument given. Forcing install of all compiled-from-source components."
    FORCE=$2
else
    FORCE=
fi

#
# check for installation
#

MOBILESIM_FOUND=`MobileSim --help | grep -m 1 "MobileSim 0.7.3" | wc -l`
if [ $MOBILESIM_FOUND -eq 1 ]
then
    echo "MobileSim 0.7.3 already installed!"
fi
# can also find via: $ whereis MobileSim

WS4PY_FOUND=`python -c "import pkg_resources; print(pkg_resources.get_distribution('ws4py').version)" | grep -m 1 -o "0.3.5" | wc -l`
# pkg_resources should give '0.3.5'
# grep should find a match and repeat it
# and wc -l should give 1 if ws4py 0.3.5 was found
if [ $WS4PY_FOUND -eq 1 ]
then
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

#
# install ROS indigo OR jade (for "ubuntu/trusty64" box)
# --> comment out for "shadowrobot/ros-indigo-desktop-trusty64" box (pre-installed)
#
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
sudo apt-get -y update
sudo apt-get -y install ros-jade-desktop-full
sudo rosdep init
su - $SCRIPTUSER -c "rosdep update;"
sudo apt-get -y install python-rosinstall

# install gazebo5, too:
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
cd /home/$SCRIPTUSER/initdeps
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get -y update
sudo apt-get -y install gazebo5
sudo apt-get -y install libgazebo5-dev

# set up gazebo 5.1 via:
sudo cp -r /usr/share/gazebo-5.0/media /usr/share/gazebo-5.1/
sudo cp -r /usr/share/gazebo-5.0/worlds /usr/share/gazebo-5.1/
# then source the gazebo directories and run gazebo...
#cd /usr/share/gazebo-5.1 ? cd /usr/share/gazebo-5.0 ?
#source /usr/share/gazebo-5.1/setup.sh 
#gazebo --verbose
# ...and then grab all the models from online that you want!

# note: this will install to the home directory of whichever user is calling the script
# so, if this script is called as user 'vagrant'
# (e.g., "sudo -u vagrant /vagrant/single_installers/install_rosstuff_setup_catkinworkspace.sh")
# then this will install under /home/vagrant
# (technically /home/vagrant/catkin_ws and catkin_ws/src

# directory should exist, but just to make sure...
sudo -u $SCRIPTUSER mkdir -p /home/$SCRIPTUSER/catkin_ws/src

# remove devel and build directories if exist already (want to catkin_make from scratch)
if [ "$FORCE" == "-f" ]
then
    rm -rf /home/$SCRIPTUSER/catkin_ws/devel
    rm -rf /home/$SCRIPTUSER/catkin_ws/build
fi

sudo apt-get -y install wget curl # for wget and possible curl use below

# install gnome-terminal for multiscript*.py runs
sudo apt-get -y install gnome-terminal

# install rosbridge
sudo apt-get -y install ros-jade-rosbridge-server

# install turtlebot libraries
sudo apt-get -y install ros-jade-joy libboost-python-dev
cd /home/$SCRIPTUSER/catkin_ws/src
git clone https://github.com/turtlebot/turtlebot.git #ros-jade-turtlebot, ros-jade-turtlebot-capabilities
git clone https://github.com/turtlebot/turtlebot_interactions.git #ros-jade-turtlebot-interactions
git clone https://github.com/turtlebot/turtlebot_apps.git #ros-jade-turtlebot-apps
git clone https://github.com/turtlebot/turtlebot_simulator.git #ros-jade-turtlebot-simulator
git clone https://github.com/turtlebot/turtlebot_msgs.git #ros-jade-turtlebot-msgs
git clone https://github.com/turtlebot/turtlebot_create.git #(includes) ros-jade-create-description
git clone https://github.com/yujinrobot/kobuki.git #(includes) ros-jade-kobuki-description, ros-jade-kobuki-node, ros-jade-kobuki-bumper2pc
sudo apt-get -y install ros-jade-ecl-core # for kobuki_keyop, need ecl_exceptions
git clone https://github.com/yujinrobot/kobuki_msgs.git # for kobuki_keyop, need kobuki_msgs
git clone https://github.com/yujinrobot/yujin_ocs.git # for kobuki_controller_tutorial, need yocs_controllers
git clone https://github.com/yujinrobot/yocs_msgs.git # for yocs_joyop, need yocs_msgs
sudo apt-get -y install ros-jade-ar-track-alvar # for yocs_ar_marker_tracking, need ar_track_alvar_msgs
sudo apt-get -y install ros-jade-base-local-planner # for yocs_navi_toolkit, need base_local_planner
sudo apt-get -y install ros-jade-move-base-msgs # for yocs_navigator, need move_base_msgs
git clone https://github.com/yujinrobot/kobuki_core.git # for kobuki_auto_docking, need kobuki_dock_drive
sudo apt-get -y install libusb-dev libftdi-dev # for kobuki_ftdi, needs <usb.h> and <ftdi.h>
sudo apt-get -y install ros-jade-ecl-mobile-robot # for kobuki_driver, need ecl_mobile_robot
git clone https://github.com/robotics-in-concert/rocon_app_platform.git #ros-jade-rocon-app-manager
sudo apt-get -y install ros-jade-openni2-launch
sudo apt-get -y install ros-jade-moveit-core ros-jade-moveit-ros ros-jade-moveit-planners-ompl #ros-jade-moveit-full
git clone https://github.com/ros-planning/moveit_pr2.git #ros-jade-moveit-full
sudo apt-get -y install ros-jade-pr2-mechanism-msgs ros-jade-pr2-controllers-msgs # for pr2_moveit_plugins, need pr2_mechanism_msgs, pr2_controllers_msgs

#sudo apt-get -y install ros-indigo-turtlebot ros-indigo-turtlebot-interactions ros-indigo-turtlebot-apps ros-indigo-turtlebot-simulator ros-indigo-turtlebot-msgs ros-indigo-joy ros-indigo-create-description ros-indigo-kobuki-description ros-indigo-kobuki-node ros-indigo-rocon-app-manager ros-indigo-kobuki-bumper2pc ros-indigo-turtlebot-capabilities ros-indigo-openni2-launch libboost-python-dev ros-indigo-moveit-full

# install (SD-Robot-Vision / ua_ros_p3dx) libraries for ./rss_git/contrib/p3dx_gazebo_mod
#sudo apt-get -y install ros-indigo-controller-manager-tests
sudo apt-get -y install ros-jade-controller-manager-tests
#sudo apt-get -y install ros-indigo-gazebo-ros-control ros-indigo-ros-controllers
sudo apt-get -y install ros-jade-ros-controllers
sudo apt-get -y install ros-jade-gazebo-ros-pkgs # does not include ros-jade-gazebo-ros-control yet... do we need it? if we do, then:
git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git # includes gazebo_ros_control...
sudo apt-get -y install ros-jade-ros-control # for gazebo_ros_control, need transmission_interface

#source /usr/share/gazebo/setup.sh ?
#gazebo

# then install the p3dx gazebo model from github
cd /home/$SCRIPTUSER/catkin_ws/src
# if need to force, then remove old directory first
if [ "$FORCE" == "-f" ]
then
    rm -rf PioneerModel
fi
if [ ! -d PioneerModel ]
then
    sudo -u $SCRIPTUSER git clone https://github.com/SD-Robot-Vision/PioneerModel.git
fi
# if need to force, then remove old directory first
if [ "$FORCE" == "-f" ]
then
    rm -rf p3dx_mover
fi
if [ ! -d p3dx_mover ]
then
    sudo -u $SCRIPTUSER git clone https://github.com/SD-Robot-Vision/p3dx_mover.git
fi
# PioneerModel/p3dx_control requires controller_manager to compile

# set up catkin workspace
cd /home/$SCRIPTUSER
sudo -u $SCRIPTUSER mkdir -p catkin_ws/src/rss_work
su - $SCRIPTUSER -c "source /home/$SCRIPTUSER/.bashrc; cd /home/$SCRIPTUSER/catkin_ws/src; source /opt/ros/jade/setup.bash; /opt/ros/jade/bin/catkin_init_workspace; cd ..; /opt/ros/jade/bin/catkin_make;"
sudo -u $SCRIPTUSER echo "source /opt/ros/jade/setup.bash" >> /home/$SCRIPTUSER/.bashrc
sudo -u $SCRIPTUSER echo "source /home/$SCRIPTUSER/catkin_ws/devel/setup.bash" >> /home/$SCRIPTUSER/.bashrc
#sudo chown -R $SCRIPTUSER:$SCRIPTUSER /home/$SCRIPTUSER/.bashrc

# install ROSARIA (reference: http://wiki.ros.org/ROSARIA/Tutorials/How%20to%20use%20ROSARIA )
cd /home/$SCRIPTUSER/catkin_ws/src
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
    su - $SCRIPTUSER -c "source /home/$SCRIPTUSER/.bashrc; cd /home/$SCRIPTUSER/catkin_ws; source devel/setup.bash; rosdep update; rosdep -y install rosaria; /opt/ros/jade/bin/catkin_make;"
    # must run "rosdep update" and "rosdep install rosaria" as $SCRIPTUSER (non-root user)
    # sudo rosdep fix-permissions # if was root, would then need to run 'rosdep update' again without sudo
fi

# install deps for MobileSim and MobileSim (references: http://robots.mobilerobots.com/wiki/MobileSim and http://robots.mobilerobots.com/MobileSim/download/current/README.html )
mkdir -p ~/initdeps
cd ~/initdeps
# If you are running Ubuntu 14.04 64-bit, install these packages first to resolve dependencies, just in case (these are useful for Matlab fonts and such, too)
sudo apt-get -y install lib32z1 lib32ncurses5 lib32bz2-1.0 xfonts-100dpi
sudo apt-get -y install wget
if [ "$FORCE" == "-f" ] || [ ! -f mobilesim_0.7.3+ubuntu12+gcc4.6_amd64.deb ]
then
    wget http://robots.mobilerobots.com/MobileSim/download/current/mobilesim_0.7.3+ubuntu12+gcc4.6_amd64.deb
fi
if [ "$FORCE" == "-f" ] || [ $MOBILESIM_FOUND -eq 0 ]
then
    sudo dpkg -i mobilesim_0.7.3+ubuntu12+gcc4.6_amd64.deb
fi

# install python WebSocket library (reference: https://ws4py.readthedocs.org/en/latest/sources/install/ )
mkdir -p ~/initdeps/rosbridgeclient
cd ~/initdeps/rosbridgeclient
# if need to force, then remove old directory first
if [ "$FORCE" == "-f" ]
then
    rm -rf WebSocket-for-Python
fi
if [ ! -d WebSocket-for-Python ]
then
    git clone https://github.com/Lawouach/WebSocket-for-Python.git
fi
cd WebSocket-for-Python
if [ "$FORCE" == "-f" ] || [ $WS4PY_FOUND -eq 0 ]
then
    sudo python setup.py install
fi

echo "End of install_rosstuff_setup_catkinworkspace.sh script!"
