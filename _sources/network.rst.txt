Network Configuration
=====================

Setting Up the Robot
--------------------
1. If this is the first time you are using the robot, please follow the Franka Robot Setup instructions located in the booklet that accompanies the robot and set the username and password for the robot. 

2. Next set the Robot Network to be ``172.16.0.2`` with a netmask of ``255.255.255.0`` as shown in the photo below.

    .. image:: imgs/franka_network.png
      :width: 600
      :alt: Franka Network Photo


Editing the Ethernet Network Configuration
------------------------------------------
1. Insert the ethernet cable from the Black Franka Control box to the Control PC.
2. Turn on the Franka Control box.
3. Go to Edit Connections in the Ubuntu Network Connections Menu of your Control PC.
4. Select the Ethernet connection that corresponds to the port that you plugged the ethernet cable into and then click edit.
5. Go to the IPv4 Settings Tab and switch from Automatic (DHCP) to Manual.
6. Add an address of ``172.16.0.1`` with netmask ``255.255.255.0``  as shown in the picture below and then click save.
    .. image:: imgs/control_pc_network.png
      :width: 600
      :alt: Franka Network Photo

7. Check to see if you can ping ``172.16.0.2`` from any terminal.
8. If any issues arise, refer to here: `https://frankaemika.github.io/docs/troubleshooting.html#robot-is-not-reachable <https://frankaemika.github.io/docs/troubleshooting.html#robot-is-not-reachable>`_ 

Entering Franka Desk
--------------------
1. Go to the web browser and type in ``https://172.16.0.2/``
2. Add an exception if there is a security certificate error.
3. Login to the robot with the correct credentials or complete setup with the robot. You should now be able to see the Franka Desk GUI in the web browser.
4. If the username and password do not work, there is a method of factory resetting the Franka located here: `https://www.franka-community.de/t/reset-admin-password-or-reset-to-factory-settings/184 <https://www.franka-community.de/t/reset-admin-password-or-reset-to-factory-settings/184>`_ 


Making sure Franka Control Interface is Installed
-------------------------------------------------
1. Go to the Settings menu of Franka Desk using the dropdown menu in the upper right corner.
2. Click on System on the left hand side.
3. Make sure Franka Control Interface (FCI) is installed as shown in the photo below.
    .. image:: imgs/franka_control_interface.png
      :width: 600
      :alt: Franka Network Photo
4. If it is not installed, our Franka-Interface will not work, so you will need to contact the distributor you purchased the Franka Panda from or Franka Emika to receive the Feature file.
5. Then you will need to press the install feature button and upload the file to the Franka Control Box. 
6. The Franka may need to be rebooted before it shows up as installed.

Now you are ready to install FrankaPy to control the robot following instructions `here <https://iamlab-cmu.github.io/frankapy>`_.
