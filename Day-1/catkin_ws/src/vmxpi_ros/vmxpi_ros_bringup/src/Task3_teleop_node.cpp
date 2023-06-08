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
#include <cmath>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


static double motoer1encoder,motoer0encoder,motoer2encoder;
static double motor1_speed,motor0_speed,motor2_speed;
static double rightSpeed , leftSpeed , backSpeed;
double motor_controller;
static double angle,angle_t;
void motor1_speed_callback(const std_msgs::Float32::ConstPtr& msg)
{
   motor1_speed = msg->data;
}
void motor0_speed_callback(const std_msgs::Float32::ConstPtr& msg)
{
   motor0_speed = msg->data;
}
void motor2_speed_callback(const std_msgs::Float32::ConstPtr& msg)
{
   motor2_speed = msg->data;
}

void  teleoperated_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
   motor_controller=msg->linear.x; 

}
void encoder1_dis_callback(const std_msgs::Float32::ConstPtr& msg)
{
   motoer1encoder = msg->data;
}

void encoder0_dis_callback(const std_msgs::Float32::ConstPtr& msg)
{
   motoer0encoder = msg->data;
}
void encoder2_dis_callback(const std_msgs::Float32::ConstPtr& msg)
{
   motoer2encoder = msg->data;
}

void angleCallback(const std_msgs::Float32::ConstPtr& msg)
{
    angle = abs(msg->data);
}
void yawCallback(const std_msgs::Float32::ConstPtr& msg)
{
    angle_t = msg->data;
}
class TeleopControl
{
   private:
  
   public:

      ros::ServiceClient set_m_speed,resetAngle,res_encoder_client;

      ros::Subscriber motor1_speed_sub,motor0_speed_sub,motor2_speed_sub; 
      ros::Subscriber encoder1_dis,encoder0_dis,encoder2_dis;
      ros::Subscriber teleoperated_sub ;   
      vmxpi_ros::MotorSpeed speed_msg;
      geometry_msgs::Twist twist_msg;
      ros::Publisher odom_pub;
      tf::TransformBroadcaster odom_broadcaster;
      ros::Subscriber  angle_sub, yawAngle_sub;
      ros::Time current_time, last_time;





   
      TeleopControl(ros::NodeHandle *nh){
         set_m_speed = nh->serviceClient<vmxpi_ros::MotorSpeed>("titan/set_motor_speed");  

         motor1_speed_sub = nh->subscribe("titan/motor1/speed", 1, motor1_speed_callback);
         motor0_speed_sub = nh->subscribe("titan/motor0/speed", 1, motor0_speed_callback);
         motor2_speed_sub = nh->subscribe("titan/motor2/speed", 1, motor2_speed_callback);

         teleoperated_sub = nh->subscribe("cmd_vel",10,teleoperated_callback);

         encoder1_dis = nh->subscribe("titan/encoder1/distance",1,encoder1_dis_callback);
         encoder0_dis = nh->subscribe("titan/encoder0/distance",1,encoder0_dis_callback);
         encoder2_dis = nh->subscribe("titan/encoder2/distance",1,encoder2_dis_callback);
         odom_pub = nh->advertise<nav_msgs::Odometry>("odom",50);
        angle_sub = nh->subscribe("navx/angle", 1, angleCallback);
        yawAngle_sub = nh->subscribe("navx/yaw", 1, yawCallback);
         resetAngle = nh->serviceClient<std_srvs::Empty>("reset_navx");
        res_encoder_client = nh->serviceClient<std_srvs::Trigger>("titan/reset_encoder");
    }

   void motorcontorl()
   {
      ros::Rate loop_rate(1);
      reset();
      while (ros::ok())
      {
        speed_msg.request.speed = motor_controller ;
        speed_msg.request.motor =3;
        set_m_speed.call(speed_msg);
        ROS_INFO("RIGHT SPEED = %f",motor_controller);
         ros::spinOnce();
        loop_rate.sleep();
      }
   }

    void reset()
    {
        std_srvs::Trigger msg1; 
        res_encoder_client.call(msg1); // Resets displacement encoders
        std_srvs::Empty msg2;
        resetAngle.call(msg2); // Resets yaw variable
    }

};


int main(int argc,char **argv)
{
    system("/usr/local/frc/bin/frcKillRobot.sh");
    ros::init(argc, argv,"teleopNode");
       ros::NodeHandle nh; //internal reference to the ROS node that the program will use to interact with the ROS system
   VMXPi vmx(true, (uint8_t)50); //realtime bool and the update rate to use for the VMXPi AHRS/IMU interface, default is 50hz within a valid range of 4-200Hz
   ros::AsyncSpinner spinner(4);
   spinner.start();

   TitanDriverROSWrapper titan(&nh, &vmx);
   
   navXROSWrapper navx(&nh, &vmx);
   TeleopControl cfg(&nh);
   cfg.motorcontorl();

   
   
   ROS_INFO("ROS SHUTDOWN");
   ros::waitForShutdown();

}