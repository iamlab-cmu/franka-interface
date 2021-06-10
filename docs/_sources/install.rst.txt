Installation
============

Requirements
------------

* A computer with Ubuntu 18.04 Realtime Kernel and at least 1 ethernet port.
* ROS Melodic
* `Google Protocol Buffer <https://developers.google.com/protocol-buffers>`_

Recommended Computer Configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* 2 Ethernet ports
* Wifi
* Intel i5-9600K or better
* 16 GB of RAM
* 256 GB SSD or more


Computer Setup Instructions
---------------------------

These instructions are assuming that you just freshly installed `Ubuntu 18.04 <https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview>`_ on the computer, which we will call the Control PC.

1. First make sure your computer is completely up to date with all packages::

    sudo apt update
    sudo apt upgrade

2. Install basic packages::

    sudo apt install curl openssh-server git vim terminator

3. Install `ROS Melodic <http://wiki.ros.org/melodic/Installation/Ubuntu>`_. Make sure that ``source /opt/ros/melodic/setup.bash`` is in your ``~/.bashrc`` file. 

Protobuf
~~~~~~~~

1. First determine the number of cores on your computer using the command::

    nproc

2. Execute the following commands::

    sudo apt-get install autoconf automake libtool curl make g++ unzip
    wget https://github.com/protocolbuffers/protobuf/releases/download/v3.11.4/protobuf-all-3.11.4.zip
    unzip protobuf-all-3.11.4.zip
    cd protobuf-3.11.4
    ./configure

3. Use the number that was previously printed out using the ``nproc`` command above and substitute it as ``N`` below::

    make -jN
    sudo make install
    sudo ldconfig


Virtual Environment
~~~~~~~~~~~~~~~~~~~

1. Install Python3.6::

    sudo apt install -y python3-distutils

2. Install Pip::

    curl https://bootstrap.pypa.io/get-pip.py | sudo -H python3.6

3. Install Virtual Environment and Other Useful Python Packages::

    sudo -H pip3.6 install numpy matplotlib virtualenv

4. Create a Virtual Environment for Franka-interface::

    virtualenv -p python3.6 franka_virtual_env

5. Enter into the Virtual Environment::

    source franka_virtual_env/bin/activate

6. How to exit the Virtual Environment::

    deactivate

Realtime Kernel
~~~~~~~~~~~~~~~

If you don't want to go through the hassle of compiling the realtime kernel yourself, feel free to download a precompiled version for **Ubuntu 18.04 ONLY** here: 

1. Simply unzip the packages into your downloads folder.

2. 

Franka-Interface Installation Steps
-----------------------------------

1. Clone the Franka-interface Repository and its Submodules::

    git clone --recurse-submodules https://github.com/iamlab-cmu/franka-interface.git
    cd franka-interface

2. To allow asynchronous gripper commands, we use the ``franka_ros`` package, so install libfranka and franka_ros using the following command::

    sudo apt install ros-melodic-libfranka ros-melodic-franka-ros

3. Clone LibFranka corresponding to your robot version. For example if your firmware is 3.x use the following command::

    bash ./bash_scripts/clone_libfranka.sh 3

4. Build LibFranka::

    bash ./bash_scripts/make_libfranka.sh

5. Build franka-interface::

    bash ./bash_scripts/make_franka_interface.sh

6. Enter the franka virtual environment (:ref:`Virtual Environment`) and then run the following commands::

    pip install catkin-tools
    bash ./bash_scripts/make_catkin.sh

7. Afterwards source the ``catkin_ws`` using the following command::

    source catkin_ws/devel/setup.bash

8. It is a good idea to add the following lines to the end of your ``~/.bashrc`` file::

    source /path/to/franka_virtual_env/franka/bin/activate
    source /path/to/franka-interface/catkin_ws/devel/setup.bash --extend
