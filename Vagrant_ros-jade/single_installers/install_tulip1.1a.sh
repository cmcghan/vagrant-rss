#!/bin/bash -e

#
# in the initialization process for vagrant, this bash script is run as user 'root' from /vagrant
#

#
# Note: Ubuntu X-Windows Desktop and ROS indigo are pre-installed
# on the "shadowrobot/ros-indigo-desktop-trusty64" base box
#

echo "Start of install_tulip1.1a.sh script!"

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

# check for version of tulip-control
TULIP_FOUND=`python -c "import pkg_resources; print(pkg_resources.get_distribution('tulip').version)" | grep -m 1 -o "1.1a-dev-unknown-commit" | wc -l`
# pkg_resources should give '1.1a-dev-unknown-commit'
# grep should find a match and repeat it
# and wc -l should give 1 if tulip 1.1a was found

if [ $TULIP_FOUND -eq 1 ]
then
    echo "tulip 1.1a already installed!"
fi

# exit script immediately if libraries are already installed
if [ "$FORCE" != "-f" ] && [ $TULIP_FOUND -eq 1 ]
then
    echo "tulip libraries already installed and up-to-date, exiting..."
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

# install glpk and cvxopt:
/vagrant/single_installers/install_glpk_cvxopt.sh $FORCE
    
# back to compilation/install directory (/root/initdeps)
cd ~/initdeps

# install gr1c:
/vagrant/single_installers/install_gr1c.sh $FORCE

# back to compilation/install directory (/root/initdeps)
cd ~/initdeps

# install tulip-control v1.1a system-wide
sudo apt-get -y install default-jre default-jdk
# if need to force, then remove old directory first
if [ "$FORCE" == "-f" ]
then
    rm -rf tulip-control-1.1a
fi
if [ "$FORCE" == "-f" ] || [ ! -f v1.1a.tar.gz ]
then
    wget https://github.com/tulip-control/tulip-control/archive/v1.1a.tar.gz
    tar xvzf v1.1a.tar.gz
fi
cd tulip-control-1.1a
sudo python setup.py install

echo "End of install_tulip1.1a.sh script!"