# joyteleop - Joystick Teleoperation Node for e.DO

This ROS Node is intended and written for the e.DO robot of Comau. The e.DO Robot of Comau is an educational robot. Therefore, this node is especially for beginners who want to see how to communicate with the robot. The task of this node is to control the robot with an x-Box 360 Controller. I hope that this node can help all those who want to work on it.

## Getting Started
  ### 1. ROS Computational Graph Level
The ROS creates a network with all its processes. This communication takes place via the ROS master. A ROS node is the process that performs the calculations. Often many nodes are used together to control different and complex functions. Each node can reach this network, subscribe to it or publish something there.

 ### 2. Communication of nodes of e.DO robot
 
 Each element has different functions in this network. The ROS also provides various tools for debugging this network.
 
 rqt_graph is a tool that graphically displays the active topics and nodes. There the nodes are displayed as squares and the topics as ellipses.
 
 rqt_topic is another ROS tool that creates a list of active topics. The following images show these tools.
 
 
