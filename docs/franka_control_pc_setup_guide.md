# Franka Control PC Setup Guide

This guide describes how to setup the software for the computer to specifically control the Franka Robot Arm. It assumes that the [control pc ubuntu setup guide](./control_pc_ubuntu_setup_guide.md) was successfully completed.

## Setting Up the Robot
If this is the first time you are using the robot, please follow the Franka Robot Setup instructions located in the booklet that accompanies the robot and set the username and password for the robot. 

## Editing the Ethernet Network Configuration
1. Insert the ethernet cable from the Franka Control box to the Control PC.
2. Turn on the Franka Control box.
3. Go to Edit Connections in the Ubuntu Network Connections Menu.
4. Select the Ethernet connection that corresponds to the port that you plugged the ethernet cable into and then click edit.
5. Go to the IPv4 Settings Tab and switch from Automatic (DHCP) to Manual.
6. Add an address of 172.16.0.1 with netmask 24 and then click save.
7. Check to see if you can ping 172.16.0.2 from any terminal.
8. If any issues arise, refer to here: https://frankaemika.github.io/docs/troubleshooting.html#robot-is-not-reachable 

## Entering Franka Desk
1. Go to the web browser and type in https://172.16.0.2/
2. Add an exception if there is a security certificate error.
3. Login to the robot with the correct credentials or complete setup with the robot.
You should now be able to see the Franka desk GUI in the web browser.
4. If the username and password do not work, there is a method of factory resetting the Franka located here: https://www.franka-community.de/t/reset-admin-password-or-reset-to-factory-settings/184

## Using the Franka With Desk
1. Unlock the physical user stop.
2. Open the Robot brakes near the bottom of the Franka Desk GUI
3. (Optional) Program the Franka Robot in Desk by creating a new program and clicking and dragging blocks and moving the robot by pressing on the two buttons near the Franka’s end effectors.

## Franka ROS Installation
1. Run the following terminal command to install the Franka ROS packages:
`sudo apt install ros-kinetic-libfranka ros-kinetic-franka-ros`

Now you are ready to return back to the main [README.md](../README.md) to continue the installation.
