#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/Float64.h"
#include <geometry_msgs/TransformStamped.h>

class odom_ecoder
{
    public:
        odom_ecoder();
        void TwistCallback(const geometry_msgs::Twist& twist);

    private:
        ros::Publisher odom_pub;
        tf::TransformBroadcaster odom_broadcaster;
        ros::Subscriber odom_sub;

        ros::NodeHandle n;

        double x,y,th,vx,vy,vth;

        ros::Time current_time, last_time;
};

odom_ecoder::odom_ecoder()
{
    x = 0.0;
    y = 0.0;
    th = 0.0;
    vx = 0;
    vy = 0;
    vth = 0;

    current_time = ros::Time::now();
    last_time = ros::Time::now();

    odom_pub = n.advertise<nav_msgs::Odometry>("odom1", 10);  
    odom_sub = n.subscribe("currant_vel", 1, &odom_ecoder::TwistCallback, this); 
}

void odom_ecoder::TwistCallback(const geometry_msgs::Twist& twist)
{
    current_time = ros::Time::now();
    // ROS_INFO("odom receive:[ %.2f, %.2f, %.2f]/n",twist.linear.x,twist.linear.y,twist.angular.z);

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (twist.linear.x *cos(th)- twist.linear.y*sin(th))* dt;
    double delta_y = (twist.linear.x *sin(th)+ twist.linear.y*cos(th))* dt;
    double delta_th = twist.angular.z * dt ;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    // geometry_msgs::TransformStamped odom_trans;
    // odom_trans.header.stamp = current_time;
    // odom_trans.header.frame_id = "odom";
    // odom_trans.child_frame_id = "base_footprint";

    // odom_trans.transform.translation.x = x;
    // odom_trans.transform.translation.y = y;
    // odom_trans.transform.translation.z = 0.0;
    // odom_trans.transform.rotation = odom_quat;

    // //send the transform
    // odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    //covariance
    odom.pose.covariance[0] = 0.001;
    odom.pose.covariance[7] = 0.001;
    odom.pose.covariance[35] = 0.001;
    //set the velocity
    odom.twist.twist.linear.x = twist.linear.x;
    odom.twist.twist.linear.y = twist.linear.y;
    odom.twist.twist.angular.z = twist.angular.z ;
    //covariance
    odom.twist.covariance[0] = 0.0001;
    odom.twist.covariance[7] = 0.0001;
    odom.twist.covariance[35] = 0.0001;
    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "ecoder");
    odom_ecoder _odom_ecoder;
    ros::spin();
    return 0;
}
