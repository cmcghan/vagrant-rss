#!/bin/bash -e
# Copyright by California Institute of Technology, University of Cincinnati
# All rights reserved. See LICENSE file at:
# https://github.com/cmcghan/vagrant-rss

#
# in the initialization process for vagrant, this bash script is run as user 'root' from /vagrant
#

echo "Start of install_MobileSim.sh script!"

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
# check for installation
#

MOBILESIM_FOUND=`MobileSim --help | grep -m 1 "MobileSim 0.7.3" | wc -l`
if [ $MOBILESIM_FOUND -eq 1 ]; then
    echo "MobileSim 0.7.3 already installed!"
fi
# can also find via: $ whereis MobileSim

#
# run installation + upgrades
#

# update all packages, because "gah!" otherwise, especially for 'rosdep' stuff later
sudo apt-get -y update
sudo apt-get -y upgrade

sudo apt-get -y install wget curl # for wget and possible curl use below

# install deps for MobileSim and MobileSim (references: http://robots.mobilerobots.com/wiki/MobileSim and http://robots.mobilerobots.com/MobileSim/download/current/README.html )
if [ $MOBILESIM_FOUND -eq 0 ]
then
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
fi


echo "End of install_MobileSim.sh script!"