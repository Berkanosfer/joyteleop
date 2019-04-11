# joyteleop - Joystick Teleoperation Node for e.DO

This ROS Node is intended and written for the e.DO robot of Comau. The e.DO Robot of Comau is an educational robot. Therefore, this node is especially for beginners who want to see how to communicate with the robot. The task of this node is to control the robot with an x-Box 360 Controller. I hope that this node can help all those who want to work on it.

The documentation in the Documentation folder shows step-by-step how to create such a node and which messages have to be sent to the robot.

![Connect](https://github.com/Berkanosfer/joyteleop/blob/master/Images/Rosconnect.JPG)

## Build Dependencies
These dependencies of the package must be installed before the package is compiled. Otherwise it won't compile anyway. :)

* [Ncurses](https://www.cyberciti.biz/faq/linux-install-ncurses-library-headers-on-debian-ubuntu-centos-fedora/) is used for asynchronous control of the joints and can be downloaded and installed online.

* [eDO_core_msgs](https://github.com/Comau/eDO_core_msgs) folder contains the messages for the communication of the robot. You can find it on the GitHub page of the Comau. This folder must also be in the /src folder.

* **C++11**

## How to install?
### Clone the package

You need to clone this repository into the /src folder of the catkin-workspace. Use catkin_make to build your directory.

```
cd ~/YOUR_WORKSPACE/src
git clone https://github.com/Berkanosfer/joyteleop.git
cd ..
catkin_make
```
### Creating an environment for e.DO

The creation of a start file serves to ensure that the environment is started correctly and all at once. So it saves the effort later on. This file is best located in the home directory. It also connects us to the robot. The IP addresses below must therefore be replaced with the correct ones depending on the situation.
```
export ROS_IP = //the own IP address of the computer
export ROS_MASTER_URI = //the IP address of the robot or the ROS master.
sudo modprobe joydev
sudo modprobe xpad
```
The last two lines activate the joystick when starting the environment.
