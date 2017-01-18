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
#

echo "Start of install_ipopt.sh script!"

#
# find O/S codename
# see: http://www.ros.org/reps/rep-0003.html
#      http://www.unixtutorial.org/commands/lsb_release/
#      http://unix.stackexchange.com/questions/104881/remove-particular-characters-from-a-variable-using-bash
#
#UCODENAME=`lsb_release -c | sed 's/Codename:\t//g'`
# cleaner version from ROS install instructions:
UCODENAME=`lsb_release -sc`
echo "Ubuntu version is: $UCODENAME"
if [ $UCODENAME == "trusty" ]; then
    ;
elif [ $UCODENAME == "xenial" ]; then
    ;
else
    echo "ERROR: Unknown Ubuntu version."
    echo "Currently, install_deps.sh supports Ubuntu 14.04 trusty and Ubuntu 16.04 xenial only."
    echo "Exiting."
    exit
fi

#
# find path of this-script-being-run
# see: http://stackoverflow.com/questions/630372/determine-the-path-of-the-executing-bash-script
#
RELATIVE_PATH="`dirname \"$0\"`"
ABSOLUTE_PATH="`( cd \"$MY_PATH\" && pwd )`"
echo "PATH of current script ($0) is: $ABSOLUTE_PATH"

#
# INPUT ARGUMENT PARSING:
#

#
# get -f (force) if given
#

# if we get an input parameter (username) then use it, else use default 'vagrant'
if [ $# -eq 1 ] && [ "$1" == "-f" ]
then
    echo "-f (force) commandline argument given. Forcing install of all compiled-from-source components."
    FORCE=$1
else
    FORCE=
fi

#
# check for installation
#
IPOPT_FOUND=`ipopt -v | grep -m 1 "Ipopt 3.12.0" | wc -l`
if [ $IPOPT_FOUND -eq 1 ]
then
    echo "Ipopt 3.12.0 already installed!"
fi

# exit script immediately if libraries are already installed
if [ "$FORCE" != "-f" ] && [ $IPOPT_FOUND -eq 1 ]
then
    echo "Ipopt libraries already installed and up-to-date, exiting..."
    exit 0
fi

#
# run installation + upgrades
#

# update all packages, because "gah!" otherwise, especially for 'rosdep' stuff later
sudo apt-get -y update
sudo apt-get -y upgrade

# start in the /root directory
cd ~
# make and move into directory for holding compilation files + downloads
mkdir -p initdeps
cd initdeps

# install python libraries for deliberative/psulu_picard (doxygen installed above)
sudo apt-get -y install gcc g++ gfortran subversion patch wget
#sudo apt-get -y install gfortran
sudo apt-get -y install doxygen
# you also need to get the 3.11.4 version for Ubuntu 14.04 from the repo, otherwise you will get:
# "ipopt: error while loading shared libraries: libipoptamplinterface.so.1: cannot open shared object file: No such file or directory"
# when you try to run "ipopt -v" to check the version (missing libraries, and yes this is weird...)
# (and yes, even installing the Ubuntu repo version afterwards solves the error message issue, so weird...)
sudo apt-get -y install coinor-libipopt-dev
# if need to force, then remove old directory first
if [ "$FORCE" == "-f" ]
then
    rm -rf Ipopt-3.12.0
fi
if [ "$FORCE" == "-f" ] || [ ! -f Ipopt-3.12.0.tgz ]
then
    wget http://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.0.tgz
    tar xvzf Ipopt-3.12.0.tgz
fi
cd Ipopt-3.12.0
cd ThirdParty/Blas && ./get.Blas
cd ../Lapack && ./get.Lapack
cd ../ASL && ./get.ASL
cd ../Mumps && ./get.Mumps
cd ../Metis && ./get.Metis
cd ../..
mkdir build && cd build
../configure --prefix=/usr/local
cd ../Ipopt/src/Common
sed -i.orig '179s/__GNUC_PATCHLEVEL__ == 3/__GNUC_PATCHLEVEL__ >= 2/' IpUtils.cpp
sed -i.orig '200s/__GNUC_PATCHLEVEL__ == 3/__GNUC_PATCHLEVEL__ >= 2/' IpUtils.cpp
cd ../../../build
make && make test && make doxydoc
sudo make install

echo "End of install_ipopt.sh script!"
