#!/bin/bash -e
# Copyright by California Institute of Technology
# All rights reserved. See LICENSE file at:
# https://github.com/cmcghan/vagrant-rss

#
# in the initialization process for vagrant, this bash script is run as user 'root' from /vagrant
#

echo "Start of install_SURF2016_deps.sh script!"
echo "input arguments: [SCRIPTUSER] [FORCE (-f)]"
echo "(note: order of [SCRIPTUSER] and -f argument can be swapped)"
echo "(note: default SCRIPTUSER is \"vagrant\")"

# set defaults for input arguments
SCRIPTUSER=vagrant
FORCE=
# if we get an input parameter (username) then use it, else use default 'vagrant'
# get -f (force) if given -- NOTE: WILL -NOT- REMOVE OR FORCE-REINSTALL ROSARIA!!!
if [ $# -lt 1 ]; then
    echo "Single username not given as commandline argument. Using default of '$SCRIPTUSER'."
    exit
else # at least 1 (possibly 2) argument(s) at commandline...
    if [ "$1" == "-f" ]; then
        echo "-f (force) commandline argument given."
        FORCE=$1
    else
        echo "Username given as commandline argument."
        SCRIPTUSER=$1
    fi
    if [ $# -gt 1 ]; then # at least 2 (possibly more) arguments at commandline...
        if [ "$2" == "-f" ]; then
            echo "-f (force) commandline argument given."
            FORCE=$2
        elif [ $SCRIPTUSER -eq "vagrant" ]; then
            echo "Username given as commandline argument."
            SCRIPTUSER=$2
        else
            echo "Username already set. Second argument ignored."
        fi
    fi
fi
echo "Will be using user $SCRIPTUSER and directories at and under /home/$SCRIPTUSER..."
if [ "$FORCE" -eq "-f" ]; then
    echo "Forcing install of all compiled-from-source components."
fi

#
# check for installation
#

#MOBILESIM_FOUND=`MobileSim --help | grep -m 1 "MobileSim 0.7.3" | wc -l`
#if [ $MOBILESIM_FOUND -eq 1 ]; then
#    echo "MobileSim 0.7.3 already installed!"
#fi
# can also find via: $ whereis MobileSim

#WS4PY_FOUND=`python -c "import pkg_resources; print(pkg_resources.get_distribution('ws4py').version)" | grep -m 1 -o "0.3.5" | wc -l`
# pkg_resources should give '0.3.5'
# grep should find a match and repeat it
# and wc -l should give 1 if ws4py 0.3.5 was found
#if [ $WS4PY_FOUND -eq 1 ]; then
#    echo "ws4py 0.3.5 already installed!"
#fi
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

# for Sandra Liu, SURF 2016


# IMU-related installation dependencies &etc.

#IMU unit (Arduino Uno + Adafruit 10-DOF IMU breakout sensor):
#
#components:
#http://www.adafruit.com/product/68
#http://www.adafruit.com/product/1604
#
#instructions for setup + software downloads
#https://learn.adafruit.com/adafruit-10-dof-imu-breakout-lsm303-l3gd20-bmp180
#https://learn.adafruit.com/adafruit-10-dof-imu-breakout-lsm303-l3gd20-bmp180/connecting-it-up
#
#alt. tutorial for different set of components:
#http://diyhacking.com/arduino-mpu-6050-imu-sensor-tutorial/
#
#alt. paper on related work:
#http://digitalcommons.calpoly.edu/cgi/viewcontent.cgi?article=1114&context=aerosp
#
#alt. IMU system (for walking/pedestrians):
#http://arxiv.org/pdf/1503.07889.pdf

# from Sandra:
#http://arduino-er.blogspot.com/2014/08/arduino-ide-error-avrdude-seropen-cant.html
#(It's also asking you to change the permissions of the ttyACM0 USB port, but that might not be possible without the Arduino connected to the computer...)
#Apparently to do the Arduino tutorials, I need access to the dialout group. Would it be possible for you to do this?
# from Cat:
#Farther down the page:
#"Didn't work for me... no /dev/ttyACM0. On my Ubuntu 14xx found out that all I needed to do was to set Tools|Port in the IDE to /dev/ttyUSB0"
#Might want to try this. If it doesn't work still, then we'll have to handle this in real-time most likely -- you plug it in and I run the necessary sudo commands while it's connected up, to change the access rights so that you can get at it. Should only need to do it the once, I think. Let me know.
# from Sandra:
#So I switched back from ttyS0 to ttyACM0 and now it just works. I'm really confused as to why though.

sudo usermod -a -G dialout $USER
sudo chmod a+rw /dev/ttyACM0

#
# other deps asked for by Ioannis and/or Sandra:
#

sudo apt-get -y install geda

#sudo apt-get -y install atom

mkdir -p ~/initdeps
cd ~/initdeps
# see: https://github.com/atom/atom/releases/latest
# if need to force, then remove old directory first
if [ "$FORCE" == "-f" ]; then
    rm -rf atom-amd64.deb
fi
if [ ! -f atom-amd64.deb ]; then
    wget https://github.com/atom/atom/releases/download/v1.8.0/atom-amd64.deb
fi
sudo dpkg --install atom-amd64.deb

sudo apt-get -y install build-essential libsqlite3-dev libreadline-dev libncurses5-dev libssl-dev libbz2-dev libgdbm-dev tk-dev
sudo apt-get -y install gcc-avr binutils-avr gdb-avr avr-libc avrdude

sudo apt-get -y install screen

echo "End of install_SURF2016_deps.sh script!"
