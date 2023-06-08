
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

double servo_angle;
static double angle,angle_t;
// Returns the angle value set by the Servo motor
void servo_angle_callback(const std_msgs::Float32::ConstPtr& msg)
{
   servo_angle = msg->data;
}
void angleCallback(const std_msgs::Float32::ConstPtr& msg)
{
    angle = abs(msg->data);
}
void yawCallback(const std_msgs::Float32::ConstPtr& msg)
{
    angle_t = msg->data;
}
class ServoController
{
   private:
   

   public:
    ros::ServiceClient resetAngle;
    vmxpi_ros::Float msg;
    ros::ServiceClient setAngle;
   ros::Subscriber servo_angle_sub;
   ros::Subscriber  angle_sub, yawAngle_sub;
    ServoController(ros::NodeHandle *nh){
    setAngle = nh->serviceClient<vmxpi_ros::Float>("channel/13/servo/set_angle");
    servo_angle_sub = nh->subscribe("channel/13/servo/angle", 1, servo_angle_callback);
    angle_sub = nh->subscribe("navx/angle", 1, angleCallback);
     yawAngle_sub = nh->subscribe("navx/yaw", 1, yawCallback);
      resetAngle = nh->serviceClient<std_srvs::Empty>("reset_navx");
   
    }

    void setServoAngle()
    {
        ros::Rate loop_rate(1);
        reset();
        vmxpi_ros::Float msg;
        while (ros::ok())
        {
        msg.request.data =angle;
        setAngle.call(msg);
         ros::spinOnce();
        loop_rate.sleep();
        }
    }

        void reset()
    {
        std_srvs::Empty msg2;
        resetAngle.call(msg2); // Resets yaw variable
    }

};




int main(int argc, char **argv)
{
   system("/usr/local/frc/bin/frcKillRobot.sh"); //Terminal call to kill the robot manager used for WPILib before running the executable.
   ros::init(argc, argv, "task4Node");

   ros::NodeHandle nh; //internal reference to the ROS node that the program will use to interact with the ROS system
   VMXPi vmx(true, (uint8_t)50); //realtime bool and the update rate to use for the VMXPi AHRS/IMU interface, default is 50hz within a valid range of 4-200Hz
   ros::AsyncSpinner spinner(4);
   spinner.start(); 
   ServoROS servo(&nh, &vmx, 13);
   navXROSWrapper navx(&nh, &vmx);


    // Use these to directly access data
   servo.GetAngle(); //returns a double;
   servo.GetMinAngle(); //returns a double
   servo.GetMaxAngle(); //returns a double

   ServoController srvController(&nh);
   srvController.setServoAngle();
   ROS_INFO("ROS SHUTDOWN");
   ros::waitForShutdown();
   return 0; 

}
