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

If you don't want to go through the hassle of compiling the realtime kernel yourself, feel free to download a precompiled version for **Ubuntu 18.04 ONLY** `here <https://drive.google.com/file/d/1Rp3_1nebSAAK8ViMMa9XX-efbWWYB7tw/view?usp=sharing>`_. 

1. Simply unzip the packages into your ``Downloads`` folder.

2. Enter the directory where you unzipped the files::

    cd Downloads/Realtime\ Kernel\ Files/

3. Install the realtime kernel by typing the following command::

    sudo dpkg -i linux-headers-5.4.3-rt1_5.4.3-rt1-1_amd64.deb linux-image-5.4.3-rt1_5.4.3-rt1-1_amd64.deb linux-libc-dev_5.4.3-rt1-1_amd64.deb

4. Restart your computer after it has finished installing the realtime kernel::

    sudo reboot

5. Once your computer has finished rebooting, ``uname -r`` should show the new kernel: 5.4.3-rt1 and ``cat /sys/kernel/realtime`` should show an output of ``1``.

6. Next you need to set realtime settings::

    sudo addgroup realtime
    sudo usermod -a -G realtime $(whoami)

7. Add the following to the end of ``/etc/security/limits.conf`` (with ``sudo gedit`` or ``sudo <your favorite editor>``)::

    @realtime soft rtprio 99
    @realtime soft priority 99
    @realtime soft memlock 102400
    @realtime hard rtprio 99
    @realtime hard priority 99
    @realtime hard memlock 102400

8. Restart your computer again::

    sudo reboot

CPU Monitoring Utilities
~~~~~~~~~~~~~~~~~~~~~~~~

Now, we want to install some utilities and files that will maintain the correct CPU governor mode. (For reference, there are usually two CPU governor modes available: ``powersave`` and ``performance``. We always want to run the Control PC in ``performance`` mode, which maintains the maximum CPU frequency. Powersave is great for laptops, not laboratory experiments. The following is adapted from `https://askubuntu.com/questions/929884/how-to-set-performance-instead-of-powersave-as-default <https://askubuntu.com/questions/929884/how-to-set-performance-instead-of-powersave-as-default>`_ and `https://askubuntu.com/questions/621184/how-to-make-cpupower-not-reset-after-each-restart <https://askubuntu.com/questions/621184/how-to-make-cpupower-not-reset-after-each-restart>`_)

1. Run the following command::

    sudo apt install indicator-cpufreq cpufrequtils

2. Restart the computer and then confirm that indicator-cpufreq starts when logged in. You should see what looks like a CPU icon in the system toolbar, with a drop-down menu that shows the current CPU governor. You can select the ``performance`` mode here, but we will now add files to do this automatically.

Run the following terminal commands:
`echo "GOVERNOR="performance"" | sudo tee /etc/default/cpufrequtils`
Defines the default CPU governor
`sudo /etc/init.d/cpufrequtils restart`
Restarts cpufrequtils so that performance mode is selected
`echo "sudo /etc/init.d/cpufrequtils restart" | sudo tee /etc/init.d/cpu.sh`
Creates cpu.sh script that restarts cpufrequtils
`sudo chmod +x /etc/init.d/cpu.sh`
Allows cpu.sh to be executable
`sudo update-rc.d cpu.sh defaults`
Allows cpu.sh to be executed at startup?
`sudo gedit /etc/rc.local`
Opens /etc/rc.local for editing (see next step for what to add)
Add the following lines to /etc/rc.local above “exit 0”:
```
sleep 90 # Give CPU startup routines time to settle.
/etc/init.d/cpu.sh
```

In short, we have defined the default CPU governor, then created several processes for Ubuntu to automatically select this governor when you log in.

Reboot.

## Testing CPU Governor Mode
It is important to test that the correct CPU governor mode is automatically selected on startup, because using the wrong mode may adversely affect communications with the robot arms and thus experiments. We want this to be automatically set correctly when logging in to the computer and not have to deal with it.

From a computer reboot, log in to the computer.
Select the indicator-cpufreq icon to display the drop-down menu, but do not select anything. We will keep the menu open during this test.
Observe that the governor is currently set to performance.
After some time after login (30-60 seconds), a system process will change this to powersave. You will see this change automatically in the menu.

However, after about 90 seconds, you should observe that the governor automatically changes back to performance.
This happens because of the commands we added to the /etc/rc.local file!

If you observe that the governor properly gets changed to performance mode, then everything has been set up correctly.


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
