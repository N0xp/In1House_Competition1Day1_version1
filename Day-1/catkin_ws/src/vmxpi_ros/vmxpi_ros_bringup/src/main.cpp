#include "TitanDriver_ros_wrapper.h"
#include "navX_ros_wrapper.h"
#include "Cobra_ros.h"
#include "Sharp_ros.h"
#include "Servo_ros.h"
#include "Ultrasonic_ros.h"
#include "IOwd_ros.h"
#include "DI_ros.h"
#include "DO_ros.h"
#include <unistd.h>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"

#include <dynamic_reconfigure/server.h>
#include <vmxpi_ros_bringup/MotorSpeedConfig.h>
#include <vmxpi_ros/Float.h>
#include <cmath>


double channel_1_V;

// Returns the channel 1 voltage value reported by the Cobra sensor
void c1_v_callback(const std_msgs::Float32::ConstPtr& msg)
{
   channel_1_V = msg->data ;
}

int main(int argc, char **argv)
{
   system("/usr/local/frc/bin/frcKillRobot.sh"); //Terminal call to kill the robot manager used for WPILib before running the executable.
   ros::init(argc, argv, "main_node");

   ros::NodeHandle nh; //internal reference to the ROS node that the program will use to interact with the ROS system
   VMXPi vmx(true, (uint8_t)50); //realtime bool and the update rate to use for the VMXPi AHRS/IMU interface, default is 50hz within a valid range of 4-200Hz
   //ros::AsyncSpinner spinner(4);
   ros::AsyncSpinner spinner(4);
   spinner.start(); 
   ros::Subscriber c1_v_sub;

   CobraROS cobra(&nh, &vmx); //default device address is 0x48 and default voltage is 5.0F
   // or can use
   // CobraROS cobra(&nh, &vmx, deviceAddress);
   // // or if sensor is using 3.3V, refVoltage(3.3F)
   // CobraROS cobra(&nh, &vmx, deviceAddress, refVoltage);

   // Use these to directly access data
   // float voltage = cobra.GetVoltage(); //returns a float
   // int raw_cobra = cobra.GetRawValue(); //returns an int

   // Subscribing to a Cobra voltage topic to access the voltage data
   c1_v_sub = nh.subscribe("cobra/c1/voltage", 1, c1_v_callback);
   
   //ros::spin() will enter a loop, pumping callbacks to obtain the latest sensor data
   ROS_INFO("ROS SHUTDOWN");
   ros::waitForShutdown();
   return 0; 

}
