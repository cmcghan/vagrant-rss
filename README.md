# vagrant-rss
Vagrantfile(s) for setting up a VM for the RSE implementation (includes installation scripts)

Installation
============

Install VirtualBox and Vagrant on your machine first before attempting to use the Vagrantfile(s) in this repository:
* VirtualBox: https://www.virtualbox.org/wiki/Downloads
* Vagrant: https://www.vagrantup.com/downloads.html

Requirements, Setup, Use
========================

See the individual Vagrantfile(s) for up-to-date information on requirements, setup, and use.

The virtual machine created by the indigo Vagrantfile requires:
* 2 CPUs, 4GB RAM, 40GB space (dynamic)
So if you want to use this Vagrantfile, you will need:
* a virtual 4-core CPU or higher, >8GB RAM, >30GB of space free

The virtual machine created by the jade Vagrantfile requires:
* 4 CPUs, 4GB RAM, 40GB space (dynamic)
So if you want to use this Vagrantfile, you will need:
* a virtual 8-core CPU or higher, >8GB RAM, >30GB of space free

It is recommended that you create and use the ROS indigo VagrantBox virtual machine, as the turtlebot libraries are supported natively in indigo, but not jade.

Note that the directory  
    `./vagrant-rss`  
is synced with  
    `/vagrant`  
in the VM.

Also note that these install scripts can be used separately on a native Ubuntu 14.04 install. To do so, try:
```
    sudo mkdir /vagrant
    sudo chown $USER:$USER /vagrant
    git clone https://github.com/cmcghan/vagrant-rss.git /vagrant
    cd vagrant-rss
    chmod +x `find . -name *.sh` # make sure .sh are executable
    cp -R * /vagrant
    cd /vagrant
    sudo su
    ./install_deps.sh SCRIPTUSER ROSVERSION
```
where SCRIPTUSER should be replaced with the name of your user account (e.g., vagrant), and ROSVERSION is the ROS distro to install (either indigo or jade).

Inside the Vagrantbox, these scripts can be rerun post-setup, after the initial `vagrant up` command, via:
```
    cd /vagrant
    sudo su
    ./install_deps.sh vagrant ROSVERSION -f
```
where ROSVERSION is the ROS distro to reinstall (either indigo or jade), and "-f" will force-reinstall the installed-from-source libraries.

For a ROS indigo Vagrantbox:
----------------------------

Note that the default vagrantfile Vagrantfile is a copy of Vagrantfile.indigo

The intended usage is:
```
    git clone https://github.com/cmcghan/vagrant-rss.git
    cd vagrant-rss
    vagrant box add shadowrobot/ros-indigo-desktop-trusty64
    vagrant up
    vagrant ssh
    cd ~/catkin_ws/src/rss_work
```

For a ROS jade Vagrantbox:
--------------------------

Note that the default vagrantfile Vagrantfile is a copy of Vagrantfile.indigo, thus you must copy the jade Vagrantfile over first before the 'vagrant up' command here.

The intended usage is:
```
    git clone https://github.com/cmcghan/vagrant-rss.git
    cd vagrant-rss/Vagrant_ros-jade
    vagrant box add ubuntu/trusty64
    cp Vagrantfile.jade Vagrantfile
    vagrant up
    vagrant ssh
    cd ~/catkin_ws/src/rss_work
```

License
=======

This is free software released under the terms of the BSD 3-Clause License. There is no warranty; not even for merchantability or fitness for a particular purpose. Consult LICENSE for copying conditions.

When code is modified or re-distributed, the LICENSE file should accompany the code or any subset of it, however small. As an alternative, the LICENSE text can be copied within files, if so desired.

Contact
=======

If you have any questions regarding the contents of this repository, please email Catharine McGhan at <cmcghan@cms.caltech.edu>.

-EOF-
