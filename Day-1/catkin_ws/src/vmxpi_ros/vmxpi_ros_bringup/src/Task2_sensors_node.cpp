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

#include <vmxpi_ros/Float.h>
#include <cmath>

double sharp_dist;
double ultrasonic_cm;
void sharp_dist_callback(const std_msgs::Float32::ConstPtr& msg)
{
   sharp_dist = msg->data;
   ROS_INFO("Distance form IR sensor to object: %f mm", sharp_dist*10);
}

void ultrasonic_cm_callback(const std_msgs::Float32::ConstPtr& msg)
{
   ultrasonic_cm = msg->data;
   ROS_INFO("Distance form US sensor to object: %f mm", ultrasonic_cm*10 );
}

int main(int argc,char **argv)
{
   system("/usr/local/frc/bin/frcKillRobot.sh"); 
    ros::init(argc,argv,"task2_node");
    ros::NodeHandle nh;
   VMXPi vmx(true, (uint8_t)50); 
   
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::Subscriber sharpDist_sub;
   ros::Subscriber ultrasonicCM_sub;
   ros::Rate loop_rate(10);

   UltrasonicROS ultrasonic(&nh, &vmx, 8, 9);   
   ultrasonic.Ultrasonic();
   SharpROS sharp(&nh, &vmx, 22);
      
        while (ros::ok())
        {
         sharpDist_sub = nh.subscribe("channel/22/sharp_ir/dist", 1, sharp_dist_callback);
         ultrasonicCM_sub = nh.subscribe("channel/9/ultrasonic/dist/cm", 1, ultrasonic_cm_callback);
         
        loop_rate.sleep();
        }


     ROS_INFO("ROS SHUTDOWN");
    ros::waitForShutdown();
    return 0;

}