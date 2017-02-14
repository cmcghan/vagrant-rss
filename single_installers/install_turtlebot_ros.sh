#!/bin/bash -e
# Copyright by California Institute of Technology, University of Cincinnati
# All rights reserved. See LICENSE file at:
# https://github.com/cmcghan/vagrant-rss

#
# in the initialization process for vagrant, this bash script is run as user 'root' from /vagrant
#

echo "Start of install_turtlebot_ros.sh script!"
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

# install turtlebot libraries
sudo apt-get -y install ros-$ROSVERSION-joy libboost-python-dev
sudo apt-get -y install ros-$ROSVERSION-openni2-launch
if [ "$ROSVERSION" -eq "indigo" ]; then
    sudo apt-get -y install ros-$ROSVERSION-turtlebot ros-$ROSVERSION-turtlebot-interactions ros-$ROSVERSION-turtlebot-apps ros-$ROSVERSION-turtlebot-simulator ros-$ROSVERSION-turtlebot-msgs  ros-$ROSVERSION-create-description ros-$ROSVERSION-kobuki-description ros-$ROSVERSION-kobuki-node ros-$ROSVERSION-rocon-app-manager ros-$ROSVERSION-kobuki-bumper2pc ros-$ROSVERSION-turtlebot-capabilities ros-$ROSVERSION-moveit-full
    # said-also-required from: http://wiki.ros.org/turtlebot/Tutorials/indigo/Debs%20Installation
    # sudo apt-get -y install ros-indigo-kobuki-ftdi ros-indigo-rocon-remocon # look like they may be included as auto-dependencies? need to check
    # sudo apt-get -y ros-indigo-rocon-qt-library ros-indigo-ar-track-alvar-msgs # look like they may be included as auto-dependencies? need to check
elif [ "$ROSVERSION" -eq "jade" ]; then
    cd /home/$SCRIPTUSER/catkin_ws/src
    if [ "$FORCE" == "-f" ]; then
        rm -rf turtlebot
        rm -rf turtlebot_interactions
        rm -rf turtlebot_apps
        rm -rf turtlebot_simulator
        rm -rf turtlebot_msgs
        rm -rf turtlebot_create
        rm -rf kobuki
        rm -rf kobuki_msgs
        rm -rf yujin_ocs
        rm -rf yocs_msgs
        rm -rf kobuki_core
        rm -rf rocon_app_platform
        rm -rf moveit_pr2
    fi
    if [ ! -d turtlebot ]; then
        sudo -u $SCRIPTUSER git clone https://github.com/turtlebot/turtlebot.git #ros-jade-turtlebot, ros-jade-turtlebot-capabilities
    fi
    if [ ! -d turtlebot_interactions ]; then
        sudo -u $SCRIPTUSER git clone https://github.com/turtlebot/turtlebot_interactions.git #ros-jade-turtlebot-interactions
    fi
    if [ ! -d turtlebot_apps ]; then
        sudo -u $SCRIPTUSER git clone https://github.com/turtlebot/turtlebot_apps.git #ros-jade-turtlebot-apps
    fi
    if [ ! -d turtlebot_simulator ]; then
        sudo -u $SCRIPTUSER git clone https://github.com/turtlebot/turtlebot_simulator.git #ros-jade-turtlebot-simulator
    fi
    if [ ! -d turtlebot_msgs ]; then
        sudo -u $SCRIPTUSER git clone https://github.com/turtlebot/turtlebot_msgs.git #ros-jade-turtlebot-msgs
    fi
    if [ ! -d turtlebot_create ]; then
        sudo -u $SCRIPTUSER git clone https://github.com/turtlebot/turtlebot_create.git #(includes) ros-jade-create-description
    fi
    if [ ! -d kobuki ]; then
        sudo -u $SCRIPTUSER git clone https://github.com/yujinrobot/kobuki.git #(includes) ros-jade-kobuki-description, ros-jade-kobuki-node, ros-jade-kobuki-bumper2pc
    fi
    sudo apt-get -y install ros-$ROSVERSION-ecl-core # for kobuki_keyop, need ecl_exceptions
    if [ ! -d kobuki_msgs ]; then
        sudo -u $SCRIPTUSER git clone https://github.com/yujinrobot/kobuki_msgs.git # for kobuki_keyop, need kobuki_msgs
    fi
    if [ ! -d yujin_ocs ]; then
        sudo -u $SCRIPTUSER git clone https://github.com/yujinrobot/yujin_ocs.git # for kobuki_controller_tutorial, need yocs_controllers
    fi
    if [ ! -d yocs_msgs ]; then
        sudo -u $SCRIPTUSER git clone https://github.com/yujinrobot/yocs_msgs.git # for yocs_joyop, need yocs_msgs
    fi
    sudo apt-get -y install ros-$ROSVERSION-ar-track-alvar # for yocs_ar_marker_tracking, need ar_track_alvar_msgs
    sudo apt-get -y install ros-$ROSVERSION-base-local-planner # for yocs_navi_toolkit, need base_local_planner
    sudo apt-get -y install ros-$ROSVERSION-move-base-msgs # for yocs_navigator, need move_base_msgs
    if [ ! -d kobuki_core ]; then
        sudo -u $SCRIPTUSER git clone https://github.com/yujinrobot/kobuki_core.git # for kobuki_auto_docking, need kobuki_dock_drive
    fi
    sudo apt-get -y install libusb-dev libftdi-dev # for kobuki_ftdi, needs <usb.h> and <ftdi.h>
    sudo apt-get -y install ros-$ROSVERSION-ecl-mobile-robot # for kobuki_driver, need ecl_mobile_robot
    if [ ! -d rocon_app_platform ]; then
        sudo -u $SCRIPTUSER git clone https://github.com/robotics-in-concert/rocon_app_platform.git #ros-jade-rocon-app-manager
    fi
    sudo apt-get -y install ros-$ROSVERSION-moveit # should include -core, -ros, -planners
    #sudo apt-get -y install ros-$ROSVERSION-moveit-core ros-$ROSVERSION-moveit-ros ros-$ROSVERSION-moveit-planners-ompl #ros-jade-moveit-full
    if [ ! -d moveit_pr2 ]; then
        sudo -u $SCRIPTUSER git clone https://github.com/ros-planning/moveit_pr2.git #ros-jade-moveit-full
    fi
    sudo apt-get -y install ros-$ROSVERSION-pr2-mechanism-msgs ros-$ROSVERSION-pr2-controllers-msgs # for pr2_moveit_plugins, need pr2_mechanism_msgs, pr2_controllers_msgs
elif [ "$ROSVERSION" -eq "kinetic" ]; then
    sudo apt-get -y install ros-$ROSVERSION-turtlebot ros-$ROSVERSION-turtlebot-interactions ros-$ROSVERSION-turtlebot-apps ros-$ROSVERSION-turtlebot-simulator ros-$ROSVERSION-turtlebot-msgs  ros-$ROSVERSION-create-description ros-$ROSVERSION-kobuki-description ros-$ROSVERSION-kobuki-node ros-$ROSVERSION-rocon-app-manager ros-$ROSVERSION-kobuki-bumper2pc ros-$ROSVERSION-turtlebot-capabilities ros-$ROSVERSION-moveit-full
    # packages that don't exist as of 2016-10-19:
    # ros-kinetic-moveit-full
    # see issues list: https://github.com/ros-planning/moveit/issues/18

#
# *** mainly untested git pulls and compiles below!!! ***
#

    # said-also-required from: http://wiki.ros.org/turtlebot/Tutorials/indigo/Debs%20Installation
    # sudo apt-get -y ros-$ROSVERSION-rocon-qt-library ros-$ROSVERSION-ar-track-alvar-msgs # not included as auto-dependencies above
    # latter is handled by specific package install below
    cd /home/$SCRIPTUSER/catkin_ws/src
    if [ "$FORCE" == "-f" ]; then
        #rm -rf rocon_qt_gui
        #rm -rf 0.7-indigo
        rm -rf moveit
        rm -rf moveit_pr2
        rm -rf pr2_controllers
        rm -rf pr2_mechanism_msgs
        rm -rf pr2_mechanism
    fi
    sudo apt-get -y install ros-$ROSVERSION-ar-track-alvar # for yocs_ar_marker_tracking, need ar_track_alvar_msgs
    # ros-indigo-rocon-qt-library ???? -- is part of rocon-qt-gui; either need indigo branch or goes w/renaming:
    #if [ ! -d rocon_qt_gui ]; then
    #sudo -u $SCRIPTUSER git clone https://github.com/robotics-in-concert/rocon_qt_gui.git # devel branch
    #fi
    #if [ ! -d 0.7-indigo ]; then
    #sudo -u $SCRIPTUSER git clone https://github.com/robotics-in-concert/rocon_qt_gui/tree/release/0.7-indigo.git # indigo version
    #fi
    # ros-kinetic-moveit-full:
    # ros-kinetic-moveit-core ros-kinetic-moveit-ros ros-kinetic-moveit-planners-ompl
    if [ ! -d moveit ]; then
        sudo -u $SCRIPTUSER git clone https://github.com/ros-planning/moveit.git #ros-kinetic-moveit-full
    fi
    # optional(?) with ros-kinetic-moveit-full:
    if [ ! -d moveit_pr2 ]; then
        #sudo -u $SCRIPTUSER git clone https://github.com/ros-planning/moveit_pr2.git #ros-"indigo"-moveit-full (for jade)
        sudo -u $SCRIPTUSER git clone -b kinetic-devel https://github.com/ros-planning/moveit_pr2.git #ros-kinetic-moveit-full
    fi
    # for pr2_moveit_plugins, need pr2_mechanism_msgs, pr2_controllers_msgs
    if [ ! -d pr2_controllers ]; then
    sudo -u $SCRIPTUSER git clone https://github.com/pr2/pr2_controllers.git # for pr2_moveit_plugins, need pr2_controllers_msgs
    fi
    if [ ! -d pr2_mechanism_msgs ]; then
    sudo -u $SCRIPTUSER git clone https://github.com/PR2/pr2_mechanism_msgs.git # for pr2_moveit_plugins, need pr2_mechanism_msgs
    fi
    if [ ! -d pr2_mechanism ]; then
    sudo -u $SCRIPTUSER git clone https://github.com/pr2/pr2_mechanism.git # adding pr2_mechanism, just in case
    fi
fi





echo "End of install_turtlebot_ros.sh script!"