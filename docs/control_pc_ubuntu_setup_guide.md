# Control PC Ubuntu Setup Guide

This guide describes how to setup the Ubuntu 16.04 Realtime Kernel and ROS for a computer to control either the Franka or UR5e arm.

## Install Ubuntu 16.04
Follow the tutorial located [here](https://tutorials.ubuntu.com/tutorial/tutorial-install-ubuntu-desktop-1604) to install Ubuntu 16.04 on your computer.

## Realtime Kernel Installation
In order to communicate with the Franka Panda Research Arm at 1000 Hz and the UR5e at 500 Hz, we need Ubuntu to be patched with the PREEMPT_RT patch. This is also known as the “realtime kernel” patch. 

First, install required dependencies:
`sudo apt install build-essential bc curl ca-certificates fakeroot gnupg2 libssl-dev lsb-release libelf-dev bison flex`

Secondly, you need to pick a mainline kernel version that has a preempt_rt [“RT”] patch. What worked best was selecting the next closest RT kernel available to what was installed on the system. (List of RT versions: https://mirrors.edge.kernel.org/pub/linux/kernel/projects/rt/) Out of the box, Ubuntu 16.04 LTS comes with kernel “4.15.0-29-generic”, although running Software Updater can update to a newer version of the 4.15.0 kernel (e.g., “4.15.0-36-generic”). So we picked 4.16.18.
You can identify what kernel version you are currently using with `uname -r`.

We will download both the mainline version of the kernel we want along with the RT patch, extract the mainline kernel and apply the RT patch, then compile the kernel and install it.

Create the directory and download the kernel files:
```
mkdir -p ~/Downloads/preempt_rt_4.16.18
cd ~/Downloads/preempt_rt_4.16.18
curl -SLO https://www.kernel.org/pub/linux/kernel/v4.x/linux-4.16.18.tar.xz
curl -SLO https://www.kernel.org/pub/linux/kernel/v4.x/linux-4.16.18.tar.sign
curl -SLO https://www.kernel.org/pub/linux/kernel/projects/rt/4.16/patch-4.16.18-rt12.patch.xz
curl -SLO https://www.kernel.org/pub/linux/kernel/projects/rt/4.16/patch-4.16.18-rt12.patch.sign
```

(In the patch directory [https://mirrors.edge.kernel.org/pub/linux/kernel/projects/rt/4.16/ in this case] you may see both “patch” and “patches” file. Use “patch” -- it’s one file that contains everything that’s needed.)
The .sign files are used to verify the files (if you’d like -- it’s optional).

Decompress tar files:
```
xz -d linux-4.16.18.tar.xz
xz -d patch-4.16.18-rt12.patch.xz
```

Apply the kernel patch:
```
tar xf linux-4.16.18.tar
cd linux-4.16.18/
patch -p1 < ../patch-4.16.18-rt12.patch
```

Start kernel configuration using the current kernel’s config:
`make oldconfig`

Select the default [hit enter] for everything EXCEPT the kernel preemption model. It will look like this:
```
Preemption Model
  1. No Forced Preemption (Server) (PREEMPT_NONE)
  2. Voluntary Kernel Preemption (Desktop) (PREEMPT_VOLUNTARY)
  3. Preemptible Kernel (Low-Latency Desktop) (PREEMPT__LL) (NEW)
  4. Preemptible Kernel (Basic RT) (PREEMPT_RTB) (NEW)
  5. Fully Preemptible Kernel (RT) (PREEMPT_RT_FULL) (NEW)
choice[1-5?]:
```
Input `5` to choose the full preemptible kernel.

Build the kernel:
`fakeroot make -j4 deb-pkg`

This will take a long time (around 2.5 hours). Grab a cup of coffee or two...
Now, we want to install the new .deb packages, but not ones with `dbg` in the file name:

Installing the kernel .deb packages:
```
cd ..
sudo dpkg -i linux-headers-4.16.18-rt12_4.16.18-rt12-1_amd64.deb linux-image-4.16.18-rt12_4.16.18-rt12-1_amd64.deb linux-libc-dev_4.16.18-rt12-1_amd64.deb
```

Now, restart. If the RT kernel has been successfully installed, the following will confirm:
`uname -r` will show the new kernel: 4.16.18-rt12
`cat /sys/kernel/realtime` should show an output of “1”

Set real-time settings
```
sudo addgroup realtime
sudo usermod -a -G realtime $(whoami)
```

Add the following to: /etc/security/limits.conf (with sudo gedit or sudo <your favorite editor>)
```
@realtime soft rtprio 99
@realtime soft priority 99
@realtime soft memlock 102400
@realtime hard rtprio 99
@realtime hard priority 99
@realtime hard memlock 102400
```

Restart one final time.

## ROS Installation
Install ROS (using the desktop-full package) using this guide: http://wiki.ros.org/kinetic/Installation/Ubuntu

## Install CPU Monitoring Utilities
Now, we want to install some utilities and files that will maintain the correct CPU governor mode. (For reference, there are usually two CPU governor modes available: powersave and performance. We always want to run the arms in performance mode, which maintains the maximum CPU frequency. Powersave is great for laptops, not laboratory experiments. The following is adapted from https://askubuntu.com/questions/929884/how-to-set-performance-instead-of-powersave-as-default and https://askubuntu.com/questions/621184/how-to-make-cpupower-not-reset-after-each-restart)

Run the following terminal command:
`sudo apt install indicator-cpufreq cpufrequtils`

Restart and then confirm that indicator-cpufreq starts when logged in. You should see what looks like a CPU icon in the system toolbar, with a drop-down menu that shows the current CPU governor. You can select the performance mode here, but we will now add files to do this automatically.

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

It is expected from here on that you will not have to worry about the CPU governor. However, I would still double check at least for now before running experiments while we are still stabilizing and testing our system setup.

If you find that something is unexpectedly changing the CPU governor back to powersave, let Tim Lee know. We will need to debug and find out what that process is.
