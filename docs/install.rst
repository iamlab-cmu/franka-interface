Installation
============

Requirements
------------

* A computer with Ubuntu 18.04 / 20.04 Realtime Kernel and at least 1 ethernet port.
* ROS Melodic / Noetic
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

These instructions are assuming that you just freshly installed `Ubuntu 18.04 / 20.04 <https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview>`_ on the computer, which we will call the Control PC.

1. First make sure your computer is completely up to date with all packages::

    sudo apt update
    sudo apt upgrade

2. Install basic packages::

    sudo apt install curl openssh-server git vim terminator

3. Install `ROS Melodic <http://wiki.ros.org/melodic/Installation/Ubuntu>`_ or `ROS Noetic <http://wiki.ros.org/noetic/Installation/Ubuntu>`_. Make sure that ``source /opt/ros/melodic/setup.bash`` or ``source /opt/ros/noetic/setup.bash`` is in your ``~/.bashrc`` file. 

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

If you don't want to go through the hassle of compiling the realtime kernel yourself, feel free to download a precompiled version for Ubuntu 18.04 `here <https://drive.google.com/file/d/1VCPCe8m0CsgBUkfk2D_89UBAVW3QKjAQ/view?usp=sharing>`_ or Ubuntu 20.04 `here <https://drive.google.com/file/d/1mPtzHEIJW0-fIsFtA8vS6zyZBbzqmVtI/view?usp=sharing>`_. Otherwise skip down to the next section first and then come back to this section and start from step 5.

1. Simply unzip the packages into your ``Downloads`` folder.

2. Enter the directory where you unzipped the files::

    cd Downloads/Realtime\ Kernel\ Files/

3. Install the realtime kernel by typing the following command. If you are using Ubuntu 18.04, switch all of the 5.15.55-rt48 to 5.4.3-rt1::

    sudo dpkg -i linux-headers-5.15.55-rt48_5.15.55-rt48-1_amd64.deb linux-image-5.15.55-rt48_5.15.55-rt48-1_amd64.deb linux-libc-dev_5.15.55-rt48-1_amd64.deb

4. Restart your computer after it has finished installing the realtime kernel::

    sudo reboot

5. Once your computer has finished rebooting, ``uname -r`` should show the new kernel: 5.15.55-rt48 or 5.4.3-rt1 and ``cat /sys/kernel/realtime`` should show an output of ``1``.

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

Compiling your own Realtime Kernel
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
In order to communicate with the Franka Panda Research Arm at 1 kHz, we need Ubuntu to be patched with the ``PREEMPT_RT`` patch. This is also known as the “realtime kernel” patch. 

1. First, install required dependencies::

    sudo apt install build-essential bc curl ca-certificates fakeroot gnupg2 libssl-dev lsb-release libelf-dev bison flex

2. Secondly, you need to pick a mainline kernel version that has a preempt_rt [“RT”] patch. What worked best was selecting the next closest RT kernel available to what was installed on the system. (List of RT versions: `https://mirrors.edge.kernel.org/pub/linux/kernel/projects/rt/ <https://mirrors.edge.kernel.org/pub/linux/kernel/projects/rt/>`_) 

3. Out of the box, Ubuntu 18.04.6 LTS comes with kernel "5.4.0". So we picked 5.4.3 `here <https://mirrors.edge.kernel.org/pub/linux/kernel/projects/rt/5.4/older/>`_. For Ubuntu 20.04.4 LTS, it comes with kernel "5.15.0" so we picked 5.15.55 `here <https://mirrors.edge.kernel.org/pub/linux/kernel/projects/rt/5.15/>`_.
You can identify what kernel version you are currently using with the command ``uname -r``.

4. We will download both the mainline version of the kernel we want along with the RT patch, extract the mainline kernel and apply the RT patch, then compile the kernel and install it.

5. Create the directory and download the kernel files (if you are using Ubuntu 18.04, we used 5.4.3)::

    mkdir -p ~/Downloads/preempt_rt_5.15.55
    cd ~/Downloads/preempt_rt_5.15.55
    curl -SLO https://mirrors.edge.kernel.org/pub/linux/kernel/v5.x/linux-5.15.55.tar.xz
    curl -SLO https://mirrors.edge.kernel.org/pub/linux/kernel/projects/rt/5.15/patch-5.15.55-rt48.patch.xz

6. (In the patch directory (`https://mirrors.edge.kernel.org/pub/linux/kernel/projects/rt/5.15/ <https://mirrors.edge.kernel.org/pub/linux/kernel/projects/rt/5.15/>`_ in this case) you may see both ``patch`` and ``patches`` file. Use ``patch`` -- it’s one file that contains everything that’s needed.)
The ``.sign`` files are used to verify the files (if you’d like -- it’s optional).

7. Decompress tar files::

    xz -d linux-5.15.55.tar.xz
    xz -d patch-5.15.55-rt48.patch.xz

8. Apply the kernel patch::

    tar xf linux-5.15.55.tar
    cd linux-5.15.55/
    patch -p1 < ../patch-5.15.55-rt48.patch

9. Follow additional kernel configuration options `here <https://docs.ros.org/en/foxy/Tutorials/Miscellaneous/Building-Realtime-rt_preempt-kernel-for-ROS-2.html>`_.

10. If you run into errors with certificates, I followed instructions `here <https://askubuntu.com/questions/1329538/compiling-the-kernel-5-11-11>`_. and `here <https://stackoverflow.com/questions/61657707/btf-tmp-vmlinux-btf-pahole-pahole-is-not-available>`_. 

11. Now, we want to install the new ``.deb`` packages, but not ones with ``dbg`` in the file name::

     cd ..
     sudo dpkg -i linux-headers-5.15.55-rt48_5.15.55-rt48-1_amd64.deb linux-image-5.15.55-rt48_5.15.55-rt48-1_amd64.deb linux-libc-dev_5.15.55-rt48-1_amd64.deb

12. Restart your computer after it has finished installing::

     sudo reboot

CPU Monitoring Utilities
~~~~~~~~~~~~~~~~~~~~~~~~

Now, we want to install some utilities and files that will maintain the correct CPU governor mode. (For reference, there are usually two CPU governor modes available: ``powersave`` and ``performance``. We always want to run the Control PC in ``performance`` mode, which maintains the maximum CPU frequency. ``Powersave`` is great for laptops, not laboratory experiments. The following is adapted from `https://askubuntu.com/questions/929884/how-to-set-performance-instead-of-powersave-as-default <https://askubuntu.com/questions/929884/how-to-set-performance-instead-of-powersave-as-default>`_ and `https://askubuntu.com/questions/621184/how-to-make-cpupower-not-reset-after-each-restart <https://askubuntu.com/questions/621184/how-to-make-cpupower-not-reset-after-each-restart>`_)

1. Run the following command::

    sudo apt install indicator-cpufreq cpufrequtils

2. Restart the computer and then confirm that ``indicator-cpufreq`` starts when logged in. You should see what looks like a CPU icon in the system toolbar, with a drop-down menu that shows the current CPU governor. You can select the ``performance`` mode here, but we will now add files to do this automatically. Run the following terminal commands in order.

3. Defining the default CPU governor::

    echo "GOVERNOR="performance"" | sudo tee /etc/default/cpufrequtils

4. Restarting ``cpufrequtils`` so that performance mode is selected::

    sudo /etc/init.d/cpufrequtils restart

5. Creating the ``cpu.sh`` script that restarts cpufrequtils::

    sudo gedit /etc/init.d/cpu.sh

6. Add the following lines to ``cpu.sh``::

    sleep 60
    sudo /etc/init.d/cpufrequtils restart

7. Enabling ``cpu.sh`` to be executable::

    sudo chmod +x /etc/init.d/cpu.sh

8. Allowing cpu.sh to be executed at startup::

    sudo update-rc.d cpu.sh defaults

9. Creating the /etc/rc.local file::

    sudo gedit /etc/rc.local

10. Add the following lines to /etc/rc.local::

     #!/bin/sh -e
     #
     # rc.local
     #
     # This script is executed at the end of each multiuser runlevel.
     # Make sure that the script will "exit 0" on success or any other
     # value on error.
     #
     # In order to enable or disable this script just change the execution
     # bits.
     #
     # By default this script does nothing.
     
     /etc/init.d/cpu.sh &
     
     exit 0

11. Make rc.local executable::

     sudo chmod +x /etc/rc.local

12. Reboot the computer::

     sudo reboot

In short, we have defined the default CPU governor, then created several processes for Ubuntu to automatically select this governor when you log in.

It is important to test that the correct CPU governor mode is automatically selected on startup, because using the wrong mode may adversely affect communications with the robot arms and thus experiments. We want this to be automatically set correctly when logging in to the computer and not have to deal with it.

From a computer reboot, log in to the computer.
Select the indicator-cpufreq icon to display the drop-down menu, but do not select anything. We will keep the menu open during this test.
Observe that the governor is currently set to performance.
After some time after login (20-40 seconds), a system process will change this to powersave. You will see this change automatically in the menu.

However, after about 60 seconds, you should observe that the governor automatically changes back to performance.
This happens because of the commands we added to the ``/etc/rc.local`` file!

If you observe that the governor properly gets changed to performance mode, then everything has been set up correctly.


Franka-Interface Installation Steps
-----------------------------------

1. Clone the Franka-interface Repository and its Submodules::

    git clone --recurse-submodules https://github.com/iamlab-cmu/franka-interface.git
    cd franka-interface

2. To allow asynchronous gripper commands, we use the ``franka_ros`` package, so install libfranka and franka_ros using the following command. Switch melodic to noetic if you are on Ubuntu 20.04::

    sudo apt install ros-melodic-libfranka ros-melodic-franka-ros

3. Clone LibFranka corresponding to your robot version. For example if your firmware is 3.x use the following command::

    bash ./bash_scripts/clone_libfranka.sh 3

4. Build LibFranka::

    bash ./bash_scripts/make_libfranka.sh

5. Build franka-interface::

    bash ./bash_scripts/make_franka_interface.sh

6. Enter the franka virtual environment (:ref:`Virtual Environment`) and then run the following commands::

    pip install catkin-tools empy
    bash ./bash_scripts/make_catkin.sh

7. Afterwards source the ``catkin_ws`` using the following command::

    source catkin_ws/devel/setup.bash

8. Add the following lines to the end of your ``~/.bashrc`` file::

    source /path/to/franka_virtual_env/franka/bin/activate
    source /path/to/franka-interface/catkin_ws/devel/setup.bash --extend
