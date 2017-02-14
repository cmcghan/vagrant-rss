# vagrant-rss
Vagrantfile(s) for setting up a VM for the RSE implementation (includes installation scripts)

Table of Contents
=================

* Installation
* Requirements, Setup, Use
  * For a ROS indigo Vagrantbox:
  * For a ROS jade Vagrantbox:
  * For a ROS kinetic Vagrantbox:
  * Shutting down the Vagrant session:
  * Choice of access to VirtualBox:
  * If you run into issues with vagrant ssh on Windows 10...
* Post-install and/or Non-Vagrantbox Use
  * Native Ubuntu 14.04 install:
  * Native Ubuntu 16.04 install:
  * Post-install updates to Vagrant VM:
* License
* Contact

Installation
============

Install VirtualBox and Vagrant on your machine first before attempting to use the Vagrantfile(s) in this repository:
* VirtualBox: https://www.virtualbox.org/wiki/Downloads
* Vagrant: https://www.vagrantup.com/downloads.html

Optional install:
* Git: https://git-scm.com/downloads
 * note that you can download and unpack the repo manually if you don't want to install git to your main O/S environment
 * https://github.com/cmcghan/vagrant-rss/archive/master.zip
 * https://github.com/cmcghan/vagrant-rss/archive/ubuntu-16.04-xenial.zip

Requirements, Setup, Use
========================

See the individual Vagrantfile(s) for up-to-date information on requirements, setup, and use.

Note that the default vagrantfile ("Vagrantfile") is a copy of Vagrantfile.indigo

It is recommended that you create and use the ROS indigo VagrantBox virtual machine, as the turtlebot libraries are supported natively in indigo, but not jade.

For a ROS indigo Vagrantbox:
----------------------------

The virtual machine created by the indigo Vagrantfile requires:
* 2 CPUs, 4GB RAM, 40GB space (dynamic)

So if you want to use this Vagrantfile, you will need:
* a virtual 4-core CPU or higher, >8GB RAM, >30GB of space free

The intended usage is:

    git clone https://github.com/cmcghan/vagrant-rss.git
    cd vagrant-rss
    vagrant box add shadowrobot/ros-indigo-desktop-trusty64
    vagrant up
    vagrant ssh
    cd ~/catkin_ws/src/rss_work


For a ROS jade Vagrantbox:
--------------------------

The virtual machine created by the jade Vagrantfile requires:
* 4 CPUs, 4GB RAM, 40GB space (dynamic)

So if you want to use this Vagrantfile, you will need:
* a virtual 8-core CPU or higher, >8GB RAM, >30GB of space free

Note that the default vagrantfile Vagrantfile is a copy of Vagrantfile.indigo, thus you must copy the jade Vagrantfile over first before the `vagrant up` command here.

The intended usage is:

    git clone https://github.com/cmcghan/vagrant-rss.git
    cd vagrant-rss
    cp Vagrantfile.jade Vagrantfile
    vagrant box add ubuntu/trusty64
    vagrant up
    vagrant ssh
    cd ~/catkin_ws/src/rss_work

For a ROS kinetic Vagrantbox:
-----------------------------

The virtual machine created by the jade Vagrantfile requires:
* 2 CPUs, 4GB RAM, 40GB space (dynamic)

So if you want to use this Vagrantfile, you will need:
* a virtual 4-core CPU or higher, >8GB RAM, >30GB of space free

Note that the default vagrantfile Vagrantfile is a copy of Vagrantfile.indigo, thus you must copy the kinetic Vagrantfile over first before the `vagrant up` command here.

The intended usage is:

    git clone -b ubuntu-16.04-xenial https://github.com/cmcghan/vagrant-rss.git
    cd vagrant-rss
    cp Vagrantfile.kinetic Vagrantfile
    vagrant box add ubuntu/xenial64
    vagrant up --provider virtualbox
    vagrant ssh
    cd ~/catkin_ws/src/rss_work
    
Shutting down the Vagrant session:
----------------------------------

When you are done using your Vagrantbox, run  
`vagrant halt`  
to shut down the VM session.

When you wish to connect to the Vagrantbox again, try:

    vagrant up
    vagrant ssh

to start the virtual machine up again.

Choice of access to VirtualBox:
-------------------------------

Note that use of `vagrant ssh` will log you in as user "vagrant", while the VirtualBox GUI will initially log you in as user "ros".

In order to work in the VirtualBox GUI as user "vagrant", first create the Vagrantbox as per the instructions above, then run

    vagrant halt

to stop the current session.

After this:
* Open your VirtualBox application.
* Select the "rss_indigo_development_machine" or "rss_jade_development_machine" or "rss_kinetic_development_machine" in the left panel, and click on "Start".

The VM session should start, and you should be logged in automatically as user "ros".

Now, to work as user "vagrant":
* Left-click on the gear symbol in the upper-right corner, and select "Log Out..."
* Click on the right-hand icon ("Log Out").
* Wait for the user menu to come up, then select user "vagrant". Default password is the login name.

You should now be logged into the VirtualBox VM as user "vagrant".

If you run into issues with vagrant ssh on Windows 10...
--------------------------------------------------------

If you try running `vagrant ssh` and get the following error message in your terminal:

    `ssh` executable not found in any directories in the %PATH% variable.

Then try the following at the commandline:

    set PATH=%PATH%;'C:\Program Files\Git\usr\bin'

If you want to make this change permanent, -carefully- perform the following:
* Go to Control Panel-> Security->System and left-click on the link 'Advanced System Settings' in the left toolbar.
* Go to the 'Advanced' tab, then left-click the 'Environment Variables...' button at the bottom of the tab.
* Under 'System Variables' (the lower box), left-click on 'Path' and then left-click on the 'Edit...' button.
* Be careful when you edit this! Click on the 'New' button, add an entry for `C:\Program Files\Git\usr\bin` and hit the enter key. The text should end up at the bottom of the list (which you want).
* Exit out of all dialog boxes.


Post-install and/or Non-Vagrantbox Use
======================================

Native Ubuntu 14.04 install:
----------------------------

Note that these install scripts (install_deps.sh, and the scripts in the single_installers directory) can be used separately on a native Ubuntu 14.04 install. To do so, try:

    sudo git clone https://github.com/cmcghan/vagrant-rss.git /vagrant
    cd /vagrant
    sudo su
    ./install_deps.sh ROSVERSION SCRIPTUSER

where **SCRIPTUSER** should be replaced with the name of your user account (e.g., **vagrant** if your username is vagrant and you want to set up the ROS catkin workspace inside your directory), and **ROSVERSION** is the ROS distro to install (either **indigo** or **jade**).

Native Ubuntu 16.04 install:
----------------------------

Note that these install scripts (install_deps.sh, and the scripts in the single_installers directory) can be used separately on a native Ubuntu 14.04 install. To do so, try:

    sudo git clone -b ubuntu-16.04-xenial https://github.com/cmcghan/vagrant-rss.git /vagrant
    cd /vagrant
    sudo su
    ./install_deps.sh ROSVERSION SCRIPTUSER

where **SCRIPTUSER** should be replaced with the name of your user account (e.g., **vagrant** if your username is vagrant and you want to set up the ROS catkin workspace inside your directory), and **ROSVERSION** is the ROS distro to install (**kinetic**).


Post-install updates to Vagrant VM:
-----------------------------------

Note that the `git clone`'d directory  
`./vagrant-rss`  
is synced with  
`/vagrant`  
in the VM.

Inside the Vagrantbox, these scripts can be rerun post-setup, after the initial `vagrant up` command, via:

    cd /vagrant
    sudo su
    ./install_deps.sh ROSVERSION vagrant -f

where **ROSVERSION** is the ROS distro to reinstall (either **indigo** or **jade** or **kinetic**), and **-f** will force-reinstall the installed-from-source libraries.

The latter is most useful when the installation scripts in the github repository have been updated -- one can simply `git pull` an updated version of the installation scripts and then rerun them to install the new library dependencies.


License
=======

This is free software released under the terms of the BSD 3-Clause License. There is no warranty; not even for merchantability or fitness for a particular purpose. Consult LICENSE for copying conditions.

When code is modified or re-distributed, the LICENSE file should accompany the code or any subset of it, however small. As an alternative, the LICENSE text can be copied within files, if so desired.


Contact
=======

If you have any questions regarding the contents of this repository, please email Catharine McGhan at <cat.mcghan@uc.edu>.

-EOF-
