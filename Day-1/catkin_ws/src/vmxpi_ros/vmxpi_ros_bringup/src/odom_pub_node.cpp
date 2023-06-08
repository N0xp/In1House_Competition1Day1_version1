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


class odom_pub
{
    private:
    public:
    ros::Publisher odome_msge;
    odom_pub(ros::NodeHandle* nh)
    {
        odome_msge = nh->advertise<nav_msgs::Odometry>("odom",50);
    }

    void pub_pose_msg()
    {
        ros::Rate loop_rate(50);
        while (ros::ok())
        {
        nav_msgs::Odometry odom;
        odom.header.frame_id = "odom";
        odom.pose.pose.position.x =1;
        odom.pose.pose.position.y =1;
        odom.pose.pose.position.z =0;

        odome_msge.publish(odom);
        ros::spinOnce();
            loop_rate.sleep();
        }
    }
};

int main(int argc,char **argv)
{
    ros::init(argc,argv,"odom_pub_node");
    ros::AsyncSpinner spinner(4);
   spinner.start();
   ros::NodeHandle nh;
   odom_pub cfg(&nh);
   cfg.pub_pose_msg();
   

    return 0;
}