#include <ros/ros.h>
#include "edo_core_msgs/MovementCommand.h"  // for move commands
#include "edo_core_msgs/JointReset.h" // for joint reset commands
#include "edo_core_msgs/JointInit.h" // for the initialization of robot
#include <sensor_msgs/Joy.h> // includes the joystick messages, so that we can listen the joy topic
#include <iostream>
#include <queue>
#include <ncurses.h>
#include <stdlib.h>
#include <stdio.h>
#include <thread>
#include <chrono>
#include <fstream>
#include <queue>
#include <string>
#include <iomanip>  

//Global variables for axes and buttons
double intturnA = 0;
double intturnB = 0;
double intturnC = 0;
double intturnD = 0;
double intturnE = 0;
double intturnF = 0;
double intturnG = 0;

//Global variables for velocity;
double vel = 1;
double velocitymax = 1.1;
double velocityup = 0;
double velocitydown = 0;
double velocitymin = 0;

//Shortcut for the robot - Fixed Values for a "jog" move
edo_core_msgs::MovementCommand createJog(){
    edo_core_msgs::MovementCommand msg;
    msg.move_command = 74;
    msg.move_type = 74;
    msg.ovr = 100;
    msg.target.data_type = 74;
    msg.target.joints_mask = 127; // Without gripper it should be 63
    msg.target.joints_data.resize(10, 0.0);
    return msg;
}

// Class definition
class eDOTeleop
{
public:
  eDOTeleop();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  ros::Publisher jog_ctrl_pub; // Definition of ROS Publisher
  ros::Subscriber joy_sub_;    // Definition of ROS Subscriber
 
};

eDOTeleop::eDOTeleop()
{
  //Joystick subscriber
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &eDOTeleop::joyCallback, this);
  //ROS Publisher to "/bridge_jog" topic
  jog_ctrl_pub = nh_.advertise<edo_core_msgs::MovementCommand>("bridge_jog", 10);
}

void eDOTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
//Callback function for the joystick
{
    int i;

    for (unsigned i = 0; i < joy->axes.size(); ++i) {
    //ROS_INFO("Axis %d is now at position %f", i, joy->axes[i]);
    }

//Redefinition of the global variables - with joystick axes and buttons

    intturnA = joy->axes[0];
    intturnB = joy->axes[1];
    intturnC = joy->axes[4];
    intturnD = joy->axes[3];
    intturnE = joy->axes[7];
    intturnF = joy->axes[6];
    intturnG = (joy->buttons[5])*(joy->axes[5]);
    /*ROS_INFO (ROS_INFO("Axis turn h %f", intturnH);)*/

    velocityup = joy->buttons[0];
    velocitydown = joy-> buttons[1];

    ros::NodeHandle nh_;
    ros::Rate loop_rate(10);
}

double velocity(double veloc)
{
  if (velocityup==1 && veloc<velocitymax)
  {
    //ROS_INFO("Vel %f", vel);
    //ROS_INFO("velocity up %f", velocityup);
    vel=veloc+0.05;
  }

  if (velocitydown==1 && veloc>velocitymin)
  {
    //ROS_INFO("Vel %f", vel);
    //ROS_INFO("velocity down %f", velocitydown);
    vel=veloc-0.05;
  }

  return veloc;
}

//Definition of the correctvalues function - so that the robot does not react
//to all movements of the joystick.
double correctvalues(double value){

  double calibrator=0.3;
  double retval=0;

  if (value > 0 && value < 0.3)
  {
    retval = 0;
  } 
  
  if (value > -0.3 && value < 0)
  {
    retval=0;
  }

  if (value == 0)
  {
    retval=0;
  } 

  if (value == -0.0)
  {
    retval=0;
  }

 if (value>calibrator && value<1.0)
    {
      retval=-1;
    } 
    
  if (value>-1.0 && value<-0.5)
  {
      retval=1;
  }

   if (value ==-1)
  {
    retval=1;
  }

  if (value ==1)
  {
     retval=-1;
  } 
    return retval;
}

int main(int argc, char** argv)
{
//Initialize "eDOTeleop" ROS Node
//ROS_INFO("Start ROS");

  ros::init(argc, argv, "eDOTeleop");
  eDOTeleop eDOTeleop;

  ros::NodeHandle nh_;
  char proceed = '\n';
  edo_core_msgs::MovementCommand msg = createJog();

  ros::Rate loop_rate(100); // Loop frequency
  ros::Publisher jog_ctrl_pub = nh_.advertise<edo_core_msgs::MovementCommand>("bridge_jog", 10);

  //ROS_INFO("Start while");
  
  //Robot startup messages
  ros::Publisher reset_pub = nh_.advertise<edo_core_msgs::JointReset>("/bridge_jnt_reset",10); //ROS Publisher to publish joint reset command
  ros::Publisher init_pub = nh_.advertise<edo_core_msgs::JointInit>("/bridge_init",10);  //ROS Publisher to publish robot init command
  std::chrono::milliseconds timespan(10000); //Time for initialization
  edo_core_msgs::JointReset reset_msg; //Definition of the messages
  edo_core_msgs::JointInit init_msg;

  std::cout << "\033[2J\033[1;1H"; // Clean the screen;
  std::this_thread::sleep_for(timespan/4); //For the joy_node;
  while (proceed!='y'){
    std::cout << "Enter 'y' to initialize 6-Axis e.DO-Robot with gripper.\n" 
              << "The process takes 10 secs. == ";
    std::cin >> proceed;   
  }

  proceed = '\n';
  init_msg.mode = 0; //Fixed values to start up the robot
  init_msg.joints_mask = 127;
  init_msg.reduction_factor = 0.0;

  while(init_pub.getNumSubscribers() == 0){
    loop_rate.sleep();
  }

  init_pub.publish(init_msg);
  ros::spinOnce();
  loop_rate.sleep();

  std::cout << "\033[2J\033[1;1H"; // Clean the screen;
  std::this_thread::sleep_for(timespan); // While e.DO starts up

  while(proceed != 'y'){
    std::cout << "Automatic motion! \n"
              << "The robot will move, type 'y' to disengage breaks == ";
    std::cin >> proceed;
  }

  proceed = 'n';
  reset_msg.joints_mask = 127; //Fixed values to disengage the breaks
  reset_msg.disengage_steps = 2000;
  reset_msg.disengage_offset = 3.5;

  while(reset_pub.getNumSubscribers() == 0){
    loop_rate.sleep();
  }
  reset_pub.publish(reset_msg);
  ros::spinOnce();
  loop_rate.sleep();
  //End of robot startup

  std::cout << "\033[2J\033[1;1H"; // Clean the screen

  //Visualisation of joystick interface
  std::cout 
    <<"____________Welcome on board!____________\n" 
    <<"|---------------------------------------|*\n"
    <<"|    ___LB___               ___RB___    |*\n"
    <<"|   /   __   \\_____________/        \\   |*\n"
    <<"|  |  //JL\\    < >    < >      (Y)   |  |*\n"
    <<"|  |  \\__//                 (X)   (B)|  |*\n"
    <<"|  |        _           __     (A)   |  |*\n"
    <<"|  |      _|^|_       //JR\\          |  |*\n"
    <<"|  |     |<   >|       \\__//         |  |*\n"
    <<"|  |       |v|                       |  |*\n"
    <<"|  |        _______/--\\_______       |  |*\n"
    <<"|  |       |                 |       |  |*\n"
    <<"|   \\     /                   \\      /  |*\n"
    <<"|    \\___/                     \\____/   |*\n"
    <<"|---------------------------------------|*\n"
    <<"You are in the joystick mode of the e.DO robot.\n"
    <<"|---------------------------------------|*\n"
    <<"For Joint 1 -> Left joystick up and down  \n"
    <<"For Joint 2 -> Left joystick left and right\n"
    <<"For Joint 3 -> Right joystick up and down\n"
    <<"For Joint 4 -> Right joystick left and right\n"
    <<"For Joint 5 -> ^ for up and v for down\n"
    <<"For Joint 6 -> < for left and > for right\n"
    <<"For gripper -> RB to open and **RB and RT** to close\n"
    <<"-----------------------------------------\n"
    <<"Button A to speed up and Button B to speed down\n"
    << std::flush;
  while(true){
  //End of the visualisation of joystick interface
  //Fixed values for a "jog" move
        msg.move_command=74;
        msg.move_type=74;
        msg.ovr=100;
        msg.target.data_type = 74;
        msg.target.joints_mask = 127;
        
        
  //With correctvalues() - Publish the messages to the robot
        msg.target.joints_data[0] = (0,(velocity(vel))*correctvalues(intturnA));
        jog_ctrl_pub.publish(msg);

        msg.target.joints_data[1] = (0,(velocity(vel))*correctvalues(intturnB));
        jog_ctrl_pub.publish(msg);

        msg.target.joints_data[2] = (0,(velocity(vel))*correctvalues(intturnC));
        jog_ctrl_pub.publish(msg);

        msg.target.joints_data[3] = (0,(velocity(vel))*correctvalues(intturnD));
        jog_ctrl_pub.publish(msg);

        msg.target.joints_data[4] = (0,(velocity(vel))*correctvalues(intturnE));
        jog_ctrl_pub.publish(msg);

        msg.target.joints_data[5] = (0,(velocity(vel))*correctvalues(intturnF));
        jog_ctrl_pub.publish(msg);
        
  //For gripper control
        msg.target.joints_data[6] = (0,(velocity(vel))*intturnG);
        jog_ctrl_pub.publish(msg);
        
        usleep(100000);
        ros::spinOnce();
  } 
}