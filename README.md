# joyteleop - Joystick Teleoperation Node for e.DO

This ROS Node is intended and written for the e.DO robot of Comau. The e.DO Robot of Comau is an educational robot. Therefore, this node is especially for beginners who want to see how to communicate with the robot. The task of this node is to control the robot with an x-Box 360 Controller. I hope that this node can help all those who want to work on it.

The PDF files in the Documentation folder shows step-by-step how to create such a node and which messages have to be sent to the robot.

![Connect](https://github.com/Berkanosfer/joyteleop/blob/master/Images/Rosconnect.JPG)

## Build Dependencies
These dependencies of the package must be installed before the package is compiled. Otherwise it won't compile anyway. :)

* [Ncurses](https://www.cyberciti.biz/faq/linux-install-ncurses-library-headers-on-debian-ubuntu-centos-fedora/) is used for asynchronous control of the joints and can be downloaded and installed online.

* [eDO_core_msgs](https://github.com/Comau/eDO_core_msgs) folder contains the messages for the communication of the robot. You can find it on the GitHub page of the Comau. This folder must also be in the /src folder.

* [sensor_msgs](https://github.com/ros/common_msgs)

* [Joystick-Drivers](https://github.com/ros-drivers/joystick_drivers)
->Alternative: ```sudo apt-get install ros-YOUR_ROS_VERSION-joy```

* **C++11**

## How to install?
### 1. Clone the package

You need to clone this repository into the /src folder of the catkin-workspace. Use catkin_make to build your directory.

```
cd ~/YOUR_WORKSPACE/src
git clone https://github.com/Berkanosfer/joyteleop.git
cd ..
catkin_make
```
### 2. Creating an environment for e.DO

![ros_edoj.sh](https://github.com/Berkanosfer/joyteleop/blob/master/Images/15.jpg)

The creation of a start file serves to ensure that the environment is started correctly and all at once. So it saves the effort later on. This file is best located in the home directory. It also connects us to the robot. The IP addresses below must therefore be replaced with the correct ones depending on the situation.
```
export ROS_IP = //the own IP address of the computer
export ROS_MASTER_URI = //the IP address of the robot or the ROS master.
sudo modprobe joydev
sudo modprobe xpad
```
The last two lines activate the joystick when starting the environment.

## Connecting to the robot
Two options are available: WLAN connection and LAN connection.
### 1. Wi-Fi Connection
You must first connect to the robot's own WLAN network and execute the start file. Make sure that the IP addresses have been replaced with the correct ones.
### 2. LAN Connection
In order to reach the robot via LAN, it must first be established an SSH connection with the following code. So that you can change the ROS_MASTER_URI and the ROS_IP.

```
$ ssh edo@10.42.0.49
```
This is the robot's default IP address. So that we can create our own environment, it is best to reserve an IP address on our router.

The password for the robot is: raspberry. With the following code the ```ministarter``` of the robot is opened:
 ```$ sudo nano ministarter ```

The corresponding lines containing the IP address must be replaced with the correct ones. After saving the new settings, the robot must be restarted.
```$ sudo reboot```
## 3. How to start the robot?
The following code is used to execute the start and launch files and therefore the two nodes should be started with a proper connection to the e.DO robot.
```
$ source ros_edoj.sh // not necessary, this is my start file
$ roslaunch joyteleop joyteleop.launch
```
### Enjoy!

## Authors
* [Berkanosfer](mailto:berkan@kuzyaka.com)

## Licence

Licenced under the MIT Licence.

#### Source of some pictures:
- Google Images
