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
// #include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

static double motor1_speed,motor0_speed,motor2_speed;
static double rightSpeed , leftSpeed , backSpeed;
double teleoperated_forward, teleoperated_side,teleoperated_rotat;
static double motoer1encoder,motoer0encoder,motoer2encoder;
static double enc1,enc2,enc0;
static double odom_x,odom_y,odom_z;
static double left_count, right_count, back_count;
static double displacey,displacex,magnitude_t,angle,angle_t;
static double PI = 3.14159265;
static double diff_x,diff_y,threshold = 0.1;
void angleCallback(const std_msgs::Float32::ConstPtr& msg)
{
    angle = abs(msg->data);
}
void yawCallback(const std_msgs::Float32::ConstPtr& msg)
{
    angle_t = msg->data;
}
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

// Callbacks for Encoder count values
// void enc0Callback(const std_msgs::Int32::ConstPtr& msg)
// {
//     right_count = msg->data;
// }
// void enc1Callback(const std_msgs::Int32::ConstPtr& msg)
// {
//     back_count = msg->data;
// }
// void enc2Callback(const std_msgs::Int32::ConstPtr& msg)
// {
//     left_count = msg->data;
// }
// void enc2Callback(const std_msgs::Int32::ConstPtr& msg)
// {
//     left_count = msg->data;
// }
void odom_msg_callback(const nav_msgs::Odometry::ConstPtr&  msg)
{
    odom_x = msg->pose.pose.position.x;
    odom_y = msg->pose.pose.position.y;
    odom_z = msg->pose.pose.position.z;

}

class odom_sub_node
{
   private:
   bool flag = true;
    double integrator, prevError, differentiator, prevMeasurement, output;
    double prev_angle = 0.0;

   public:
      ros::ServiceClient set_m_speed;
      ros::ServiceClient resetAngle, res_encoder_client, stop_motors_client, enable_client,disable_client;

      ros::Subscriber motor1_speed_sub,motor0_speed_sub,motor2_speed_sub; 
      ros::Subscriber encoder1_dis,encoder0_dis,encoder2_dis;
      ros::Subscriber  angle_sub, yawAngle_sub;
      ros::Subscriber enc1_sub,enc0_sub,enc2_sub;
      ros::Subscriber odom_cordi;
     ros::Publisher displacex_pub, displacey_pub, magnitude_pub, lmotor_PID_pub, rmotor_PID_pub, bmotor_PID_pub, error_pub ;
          ros::Publisher odom_pub;
    //   tf::TransformBroadcaster odom_broadcaster;
      vmxpi_ros::MotorSpeed speed_msg;
  ros::Time current_time, last_time;
         double tau = 0.02, T = 0.02;
    //double tolerance = 1.0; // Tolerance of Encoder distances(mm) and angle(deg)
         double kP, kI, kD, error;
         double limMin = -1.0, limMax = 1.0, limMinInt = -0.5, limMaxInt = 0.5;
         bool atSetpoint;

      odom_sub_node(ros::NodeHandle *nh){
        set_m_speed = nh->serviceClient<vmxpi_ros::MotorSpeed>("titan/set_motor_speed");  
        resetAngle = nh->serviceClient<std_srvs::Empty>("reset_navx");
        res_encoder_client = nh->serviceClient<std_srvs::Trigger>("titan/reset_encoder");
        stop_motors_client = nh->serviceClient<std_srvs::Trigger>("titan/stop_motors");
        enable_client = nh->serviceClient<std_srvs::Trigger>("titan/enable");
        disable_client = nh->serviceClient<std_srvs::Trigger>("titan/disable");
        odom_cordi = nh->subscribe("odom",1,odom_msg_callback);
         motor1_speed_sub = nh->subscribe("titan/motor1/speed", 1, motor1_speed_callback);
         motor0_speed_sub = nh->subscribe("titan/motor0/speed", 1, motor0_speed_callback);
         motor2_speed_sub = nh->subscribe("titan/motor2/speed", 1, motor2_speed_callback);

         encoder1_dis = nh->subscribe("titan/encoder1/distance",1,encoder1_dis_callback);
         encoder0_dis = nh->subscribe("titan/encoder0/distance",1,encoder0_dis_callback);
         encoder2_dis = nh->subscribe("titan/encoder2/distance",1,encoder2_dis_callback);
        //  odom_pub = nh->advertise<nav_msgs::Odometry>("odom/drivebase",50);
        // enc0_sub = nh->subscribe("titan/encoder0/count", 1, enc0Callback);
        // enc1_sub = nh->subscribe("titan/encoder1/count", 1, enc1Callback);
        // enc2_sub = nh->subscribe("titan/encoder2/count", 1, enc2Callback);
         angle_sub = nh->subscribe("navx/angle", 1, angleCallback);
        yawAngle_sub = nh->subscribe("navx/yaw", 1, yawCallback);

        displacex_pub = nh->advertise<std_msgs::Float32>("displace_x", 1);
        displacey_pub = nh->advertise<std_msgs::Float32>("displace_y", 1);
        magnitude_pub = nh->advertise<std_msgs::Float32>("magnitude", 1);

        lmotor_PID_pub = nh->advertise<std_msgs::Float32>("lmotor_PID", 1);
        rmotor_PID_pub = nh->advertise<std_msgs::Float32>("rmotor_PID", 1);
        bmotor_PID_pub = nh->advertise<std_msgs::Float32>("bmotor_PID", 1);
        error_pub = nh->advertise<std_msgs::Float32>("error", 1);


      }
      void setPID(double Kp, double Ki, double Kd)
    {
        kP = Kp;
        kI = Ki;
        kD = Kd;
    }
        void PubDisplacements()
    {
        std_msgs::Float32 msg;
        msg.data = displacex;
        displacex_pub.publish(msg);
        msg.data = displacey;
        displacey_pub.publish(msg);
        msg.data = magnitude_t;
        magnitude_pub.publish(msg);

        msg.data = leftSpeed;
        lmotor_PID_pub.publish(msg);

        msg.data = rightSpeed;
        rmotor_PID_pub.publish(msg);

        msg.data = backSpeed;
        bmotor_PID_pub.publish(msg);

        msg.data = error;
        error_pub.publish(msg);
    }

    double calculate(double setPoint, double measurement, double tolerance)
    {
        /*
         * Error
         */
        error = setPoint - measurement;

        /*
         * Setpoint check
         */
        if (abs(error) <= tolerance)
        {
            atSetpoint = true;
            return 0.0;
        }
        else
        {
            atSetpoint = false;
        }

        /*
         * Proportional
         */
        double proportional = kP * error;

        /*
         * Integral
         */
        integrator = integrator + 0.5 * kI * T * (error + prevError);

        /*
         * Anti Wind up
         */
        if (integrator > limMaxInt)
        {
            integrator = limMaxInt;
        }
        else if (integrator < limMinInt)
        {
            integrator = limMinInt;
        }

        /*
         * Band limit derivative
         */
        differentiator = -(2.0 * kD * (measurement - prevMeasurement) + (2.0 * tau - T) * differentiator) / (2.0 * tau + T);

        /*
         * Compute
         */
        output = proportional + integrator + differentiator;

        /*
         * Clamp
         */
        if (output > limMax)
        {
            output = limMax;
        }
        else if (output < limMin)
        {
            output = limMin;
        }

        /*
         * Store variables
         */
        prevError = error;
        prevMeasurement = measurement;

        /*
         * Return final value
         */
        return output;
    }
    void PIDReset()
    {
        kP = 0.0;
        kI = 0.0;
        kD = 0.0;
        error = 0.0;
        integrator = 0.0;
        prevError = 0.0;
        differentiator = 0.0;
        prevMeasurement = 0.0;
        atSetpoint = false;
        output = 0.0;
    }


     void holonomicDrive(double x, double y, double z)
    {
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
    }

   void motorcontorl()
   {
      speed_msg.request.speed = rightSpeed ;
      speed_msg.request.motor =1;
      set_m_speed.call(speed_msg);
  

      speed_msg.request.speed = leftSpeed ;
      speed_msg.request.motor =0;
      set_m_speed.call(speed_msg);
 

      speed_msg.request.speed = backSpeed ;
      speed_msg.request.motor =2;
      set_m_speed.call(speed_msg);
    
   }

    void reset()
    {
        std_srvs::Trigger msg1;
        stop_motors_client.call(msg1); // Stops motors
        res_encoder_client.call(msg1); // Resets displacement encoders
        std_srvs::Empty msg2;
        resetAngle.call(msg2); // Resets yaw variable
    }

    void encoder2dist()
    {
        //Displace forward and back
        displacey = ((motoer0encoder * (sqrt(3) / 2)) + ((motoer1encoder * (sqrt(3) / 2)) * -1)) * -0.68;
        //displacey = (left_encoder + right_encoder) / -1.5;
        //displacey = left_encoder * (sqrt(3) / 2);
        displacey /= 1000;

        //Dispace left and right
        //displacex = back_encoder;
        displacex = (motoer2encoder + (motoer0encoder * -0.5) + (motoer1encoder * -0.5)) * 0.70;
        displacex /= 1000;

        magnitude_t = sqrt(pow(abs(displacex), 2.0) + pow(abs(displacey), 2.0));
    }

    void setMovements(double magnitude, double Angle)
    {
        if (prev_angle > Angle)
        {
            Angle = Angle - prev_angle + 360.0;
        }
        else
            Angle = Angle - prev_angle;

        double target_x_displace = magnitude * cos(abs(Angle) * PI / 180.0);
        double target_y_displace = magnitude * sin(abs(Angle) * PI / 180.0);

        ros::Rate loop_rate(50);
        while (ros::ok())
        {

            holonomicDrive(0.0, 0.0, 0.0); //Check step to zero the motors

            if (flag == true)
            {
                reset();
                displacex = 0;
                displacey = 0;
                angle = 0;
                PIDReset();
                flag = false;
            }
            
            setPID(0.95, 0.0, 0.001);
            double x_drive = calculate(target_x_displace, displacex, 0.001);
            
            //setPID(0.9, 0.0, 0.005);
            double y_drive = calculate(target_y_displace, displacey, 0.001);
            
            setPID(0.050, 0.0, 0.0);
            double angle_drive = 0.02;

            holonomicDrive(x_drive, y_drive, angle_drive);
            
            motorcontorl();
           PubDisplacements();

            if (magnitude_t >= abs(magnitude))
            {
                displacex = 0;
                displacey = 0;
                angle = 0;
                PIDReset();
                reset();
                flag = true;
                break;
            }
    //         geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(angle);
    //         geometry_msgs::TransformStamped odom_trans;
    //      odom_trans.header.stamp = current_time;
    //       odom_trans.header.frame_id = "odom";
    //      odom_trans.child_frame_id = "base_link";

    //      odom_trans.transform.translation.x = displacex;
    //      odom_trans.transform.translation.y = displacey;
    //     odom_trans.transform.translation.z = 0.0;
    //      odom_trans.transform.rotation = odom_quat;
    //          //send the transform
    // odom_broadcaster.sendTransform(odom_trans);

    // //next, we'll publish the odometry message over ROS
    // nav_msgs::Odometry odom;
    // odom.header.stamp = current_time;
    // odom.header.frame_id = "odom";

    // //set the position
    // odom.pose.pose.position.x = displacex;
    // odom.pose.pose.position.y = displacey;
    // odom.pose.pose.position.z = 0.0;
    // odom.pose.pose.orientation = odom_quat;

    // //set the velocity
    // odom.child_frame_id = "base_link";
    // odom.twist.twist.linear.x = teleoperated_forward;
    // odom.twist.twist.linear.y = teleoperated_side;
    // odom.twist.twist.angular.z = teleoperated_rotat;

    // //publish the message
    // odom_pub.publish(odom);

    // last_time = current_time;

            ros::spinOnce();
            loop_rate.sleep();
        }
    }
void drive_odom_msg()
{

        double magnitude = sqrt(odom_x*odom_x + odom_y*odom_y);
        ros::Rate loop_rate(50);
        while (ros::ok())
        {
            holonomicDrive(0.0, 0.0, 0.0); //Check step to zero the motors

            if (flag == true)
            {
                reset();
                displacex = 0;
                displacey = 0;
                angle = 0;
                PIDReset();
                flag = false;
            }
            
            setPID(0.95, 0.0, 0.001);
            double x_drive = calculate(odom_x, displacex, 0.001);
            
            //setPID(0.9, 0.0, 0.005);
            double y_drive = calculate(odom_y, displacey, 0.001);
            
            setPID(0.050, 0.0, 0.0);
            double angle_drive = odom_z;

            holonomicDrive(x_drive, y_drive, angle_drive);
            
            motorcontorl();
            encoder2dist();
           PubDisplacements();
            diff_x = odom_x + displacex;
            diff_y = odom_y + displacey;
            ROS_INFO("diff_x = %f // diff_y = %f",diff_x,diff_y);
            ROS_INFO("odom_x = %f // odom_y = %f",odom_x,odom_y);
            if (abs(diff_x)<=threshold && abs(diff_y)<=threshold)
            {
                displacex = 0;
                displacey = 0;
                angle = 0;
                PIDReset();
                reset();
                flag = true;
                break;
            }  
    //         geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(angle);
    //         geometry_msgs::TransformStamped odom_trans;
    //      odom_trans.header.stamp = current_time;
    //       odom_trans.header.frame_id = "odom";
    //      odom_trans.child_frame_id = "base_link";

    //      odom_trans.transform.translation.x = displacex;
    //      odom_trans.transform.translation.y = displacey;
    //     odom_trans.transform.translation.z = 0.0;
    //      odom_trans.transform.rotation = odom_quat;
    //          //send the transform
    // odom_broadcaster.sendTransform(odom_trans);

    // //next, we'll publish the odometry message over ROS
    // nav_msgs::Odometry odom;
    // odom.header.stamp = current_time;
    // odom.header.frame_id = "odom";

    // //set the position
    // odom.pose.pose.position.x = displacex;
    // odom.pose.pose.position.y = displacey;
    // odom.pose.pose.position.z = 0.0;
    // odom.pose.pose.orientation = odom_quat;

    // //set the velocity
    // odom.child_frame_id = "base_link";
    // odom.twist.twist.linear.x = rightSpeed;
    // odom.twist.twist.linear.y = leftSpeed;
    // odom.twist.twist.angular.z = backSpeed;

    // //publish the message
    // odom_pub.publish(odom);

    last_time = current_time;
 
              ros::spinOnce();
            loop_rate.sleep();
        }
       
        }
    



void callback(vmxpi_ros_bringup::MotorSpeedConfig &config, uint32_t level) {
        std_srvs::Trigger msg;
        if (config.enabled)
            enable_client.call(msg);
        else
            disable_client.call(msg);
        for (int i = 0; i < 1; i++)
        {
             reset();
             drive_odom_msg();
             ros::Duration(1.5).sleep();
            //  setMovements(1.0, 0.0); // (magnitude (m), angle (deg))
            //  ros::Duration(1.5).sleep(); // sleep for 1 second
             
            //  setMovements(1.0, 90.0);
            //  ros::Duration(1.5).sleep(); // sleep for 1 second
             
            //  setMovements(sqrt(2.0), 270.0);

             ros::Duration(1.5).sleep(); // sleep for 1 second
             
        }
    }
};

// Returns the speed of motor 1


int main(int argc, char **argv)
{
   system("/usr/local/frc/bin/frcKillRobot.sh"); //Terminal call to kill the robot manager used for WPILib before running the executable.
   ros::init(argc, argv, "drive_enc_node");

   ros::NodeHandle nh; //internal reference to the ROS node that the program will use to interact with the ROS system
   
   VMXPi vmx(true, (uint8_t)50); //realtime bool and the update rate to use for the VMXPi AHRS/IMU interface, default is 50hz within a valid range of 4-200Hz
   ros::AsyncSpinner spinner(4);
   spinner.start();

   TitanDriverROSWrapper titan(&nh, &vmx);
   odom_sub_node cfg(&nh);
    dynamic_reconfigure::Server<vmxpi_ros_bringup::MotorSpeedConfig> server;
    dynamic_reconfigure::Server<vmxpi_ros_bringup::MotorSpeedConfig>::CallbackType f;
    f = boost::bind(&odom_sub_node::callback, &cfg, _1, _2);
    server.setCallback(f);
   
   
   ROS_INFO("ROS SHUTDOWN");
   ros::waitForShutdown();

}
