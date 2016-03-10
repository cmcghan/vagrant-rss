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

For a ROS indigo Vagrantbox:
----------------------------

The directory  
    `./vagrant-rss/Vagrant_ros-indigo`  
is synced with  
    `/vagrant`  
in the VM.

The intended usage is:
```
    git clone https://github.com/cmcghan/vagrant-rss.git
    cd vagrant-rss/Vagrant_ros-indigo
    vagrant box add shadowrobot/ros-indigo-desktop-trusty64
    vagrant up
    vagrant ssh
    cd ~/catkin_ws/src/rss_work
```

For a ROS jade Vagrantbox:
--------------------------

The directory  
    `./vagrant-rss/Vagrant_ros-jade`  
is synced with  
    `/vagrant`  
in the VM.

The intended usage is:
```
    git clone https://github.com/cmcghan/vagrant-rss.git
    cd vagrant-rss/Vagrant_ros-jade
    vagrant box add ubuntu/trusty64
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
