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
double teleoperated_forward, teleoperated_side,teleoperated_rotat;
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
   teleoperated_forward=msg->linear.x; 
   teleoperated_side=msg->linear.y;
   teleoperated_rotat=msg->angular.z;
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
      double TransformationMatrix[3][3];
      double tempTransformationMatrix[3][3];
      double result[3][3];
      double newx;
      double newy;
      double newz;
      double displacey;
      double displacex;
      ros::ServiceClient set_m_speed,resetAngle,res_encoder_client;
      //motor 1 ::right , moto,r 0 ::left , motor 2 :: back
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
      //   identaty of hte transformation matrix
        for(int i =0; i<3; i++)
        {
         for(int j =0; j<3; j++)
         {
            if (i == j)
               TransformationMatrix[i][j]=1;
         }
        }
   
       }

      double avarage_enc_distance()
      {
         return (((motoer1encoder + motoer0encoder)/2)*1.134);
      }

      
      void holonomicDrive(double x, double y, double z)
    {
      //   rightSpeed = (((x/3)- (y/sqrt(3))+z)*sqrt(3));
      //   leftSpeed = (((x/3)+ (y/sqrt(3))+z)*sqrt(3));
      //   backSpeed = (((-2*x/3)+z)*sqrt(3));
         rightSpeed = (x / 2) + (-(y * (sqrt(3) / 2))) + z;
        leftSpeed = (x / 2) + (y * sqrt(3) / 2) + z;
        backSpeed = -x + z;


        double max = abs(rightSpeed);
        if (abs(leftSpeed) > max)
        {
            max = abs(leftSpeed);
        }
        if (abs(backSpeed) > max)
        {
            max = abs(backSpeed);
        }
        if (max > 1)
        {
            rightSpeed /= max;
            leftSpeed /= max;
            backSpeed /= max;
        }

         double cosThata = cos(z);
         double sinThata = sin(z);
         tempTransformationMatrix[0][0] = cosThata;
         tempTransformationMatrix[0][1] = -sinThata;
         tempTransformationMatrix[0][2] = x;
         tempTransformationMatrix[1][0] = sinThata;
         tempTransformationMatrix[1][1] = cosThata;
         tempTransformationMatrix[1][2] = y;
         tempTransformationMatrix[2][0] = 0.0;
         tempTransformationMatrix[2][1] = 0.0;
         tempTransformationMatrix[2][2] = 1;
      // // matricis multiplication    
      for (int i = 0; i<3; i++)
      {
         for (int j = 0; j<3; j++)
         {
            result[i][j] = 0.0;
            for(int k = 0; k<3;k++)
            {
               result[i][j] +=TransformationMatrix[i][k] * tempTransformationMatrix[k][j];
            }
         }

      }
      // matrix swap
      for (int i = 0; i <3; i++)
      {
         for (int j = 0 ; j < 3;j++)
         {
            TransformationMatrix[i][j] = tempTransformationMatrix[i][j];
         }
      }


      newx += result[0][2];
      newy += result[1][2];

      displacey = ((motoer0encoder * (sqrt(3) / 2)) + ((motoer1encoder * (sqrt(3) / 2)) * -1)) * -0.68;
      displacey /= 1000;
        displacex = (motoer2encoder + (motoer0encoder * -0.5) + (motoer1encoder * -0.5)) * 0.70;
        displacex /= 1000;


    }

   void motorcontorl()
   {
      speed_msg.request.speed = rightSpeed ;
      speed_msg.request.motor =1;
      set_m_speed.call(speed_msg);
      ROS_INFO("RIGHT SPEED = %f",rightSpeed);

      speed_msg.request.speed = leftSpeed ;
      speed_msg.request.motor =0;
      set_m_speed.call(speed_msg);
      ROS_INFO("LEFT SPEED = %f",leftSpeed);

      speed_msg.request.speed = backSpeed ;
      speed_msg.request.motor =2;
      set_m_speed.call(speed_msg);
      ROS_INFO("BACK SPEED = %f",backSpeed);
   }

    void reset()
    {
        std_srvs::Trigger msg1; 
        res_encoder_client.call(msg1); // Resets displacement encoders
        std_srvs::Empty msg2;
        resetAngle.call(msg2); // Resets yaw variable
    }

   void moving_motor_teleop()
   {
      current_time = ros::Time::now();
      ros::Rate loop_rate(1);
      reset();
      
      while (ros::ok())
      {
         holonomicDrive(teleoperated_forward,teleoperated_side,teleoperated_rotat);
           //since all odometry is 6DOF we'll need a quaternion created from yaw
            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(angle);
            motorcontorl();  


         geometry_msgs::TransformStamped odom_trans;
         odom_trans.header.stamp = current_time;
          odom_trans.header.frame_id = "odom";
         odom_trans.child_frame_id = "base_link";

         odom_trans.transform.translation.x = displacex;
         odom_trans.transform.translation.y = displacey;
        odom_trans.transform.translation.z = 0.0;
         odom_trans.transform.rotation = odom_quat;
             //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = displacex;
    odom.pose.pose.position.y = displacey;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = teleoperated_forward;
    odom.twist.twist.linear.y = teleoperated_side;
    odom.twist.twist.angular.z = teleoperated_rotat;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
      ros::spinOnce();
      loop_rate.sleep();
         }
   }
   
   
};

// Returns the speed of motor 1


int main(int argc, char **argv)
{
   system("/usr/local/frc/bin/frcKillRobot.sh"); //Terminal call to kill the robot manager used for WPILib before running the executable.
   ros::init(argc, argv, "drive_keyboard_teleop_node");

   ros::NodeHandle nh; //internal reference to the ROS node that the program will use to interact with the ROS system
   VMXPi vmx(true, (uint8_t)50); //realtime bool and the update rate to use for the VMXPi AHRS/IMU interface, default is 50hz within a valid range of 4-200Hz
   ros::AsyncSpinner spinner(4);
   spinner.start();

   TitanDriverROSWrapper titan(&nh, &vmx);
   
   navXROSWrapper navx(&nh, &vmx);
   TeleopControl cfg(&nh);
   cfg.moving_motor_teleop();
   
   
   ROS_INFO("ROS SHUTDOWN");
   ros::waitForShutdown();

}
