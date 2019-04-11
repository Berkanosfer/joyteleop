# joyteleop - Joystick Teleoperation Node for e.DO

This ROS Node is intended and written for the e.DO robot of Comau. The e.DO Robot of Comau is an educational robot. Therefore, this node is especially for beginners who want to see how to communicate with the robot. The task of this node is to control the robot with an x-Box 360 Controller. I hope that this node can help all those who want to work on it.

The documentation in the Documentation folder shows step-by-step how to create such a node and which messages have to be sent to the robot.

![Connect](https://github.com/Berkanosfer/joyteleop/blob/master/Images/Rosconnect.JPG)

## Build Dependencies
These dependencies of the package must be installed before the package is compiled. Otherwise it won't compile anyway. :)

* [Ncurses](https://www.cyberciti.biz/faq/linux-install-ncurses-library-headers-on-debian-ubuntu-centos-fedora/) is used for asynchronous control of the joints and can be downloaded and installed online.

* [eDO_core_msgs](https://github.com/Comau/eDO_core_msgs) folder contains the messages for the communication of the robot. You can find it on the GitHub page of the Comau.

* **C++11**

