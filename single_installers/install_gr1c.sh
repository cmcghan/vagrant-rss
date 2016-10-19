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

echo "Start of install_gr1c.sh script!"

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
GR1C_FOUND=`gr1c -V | grep -m 1 "gr1c 0.10.1" | wc -l`
if [ $GR1C_FOUND -eq 1 ]
then
    echo "gr1c 0.10.1 already installed!"
fi

# exit script immediately if libraries are already installed
if [ "$FORCE" != "-f" ] && [ $GR1C_FOUND -eq 1 ]
then
    echo "gr1c libraries already installed and up-to-date, exiting..."
    exit 0
fi

#
# find path of this-script-being-run
# see: http://stackoverflow.com/questions/630372/determine-the-path-of-the-executing-bash-script
#
RELATIVE_PATH="`dirname \"$0\"`"
ABSOLUTE_PATH="`( cd \"$MY_PATH\" && pwd )`"
echo "PATH of current script ($0) is: $ABSOLUTE_PATH"

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

# install gr1c:
sudo apt-get -y install curl # for curl use below
sudo apt-get -y install python-numpy python-pyparsing python-scipy python-cvxopt python-networkx python-numpy-doc python-networkx-doc python-matplotlib python-matplotlib-data python-matplotlib-doc python-pydot graphviz graphviz-doc python-pygraphviz python-scitools
sudo apt-get -y install python-dev build-essential python-pip ipython ipython-notebook python-pandas python-sympy python-nose libblas-dev liblapack-dev gfortran python-glpk glpk-utils libglpk-dev libglpk36 swig libgmp3-dev
sudo apt-get -y install bison flex
sudo apt-get -y install default-jre
gpg --keyserver pgp.mit.edu --recv-keys 03B40F63
CUDDVERSION=2.5.0
GR1CVERSION=0.10.1
# if need to force, then remove old directory first
if [ "$FORCE" == "-f" ]
then
    rm -rf gr1c-$GR1CVERSION
fi
if [ "$FORCE" == "-f" ] || [ ! -f gr1c-$GR1CVERSION.tar.gz ] || [ ! -f gr1c-$GR1CVERSION.tar.gz.sig ]
then
    curl -sO http://vehicles.caltech.edu/snapshots/gr1c/gr1c-$GR1CVERSION.tar.gz
    curl -sO http://vehicles.caltech.edu/snapshots/gr1c/gr1c-$GR1CVERSION.tar.gz.sig
    gpg --verify gr1c-$GR1CVERSION.tar.gz.sig
    FILECHECKSUM=`shasum -a 256 gr1c-$GR1CVERSION.tar.gz|cut -d ' ' -f1`
    if [ $FILECHECKSUM != '73699369ee55b95aeb3742504e27676491b6d23db176e7e84c266e1a4845c6a3' ]
    then
        echo "Checksum for the gr1c tarball does not have expected value."
        rm gr1c-$GR1CVERSION.tar.gz
        false
    fi
    tar -xzf gr1c-$GR1CVERSION.tar.gz
fi
cd gr1c-$GR1CVERSION
./get-deps.sh
make cudd
make all
make check
sudo make install

echo "End of install_gr1c.sh script!"
