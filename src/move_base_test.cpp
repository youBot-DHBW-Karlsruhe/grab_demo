#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv) {
    ROS_INFO("Initializing ros node and publisher");
    ros::init(argc, argv, "move_base_test_node");
    ros::NodeHandle node;

    ros::Publisher pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    sleep(2);

/*
    // create a msg and publish it
    ROS_INFO("move base forward");
    geometry_msgs::Twist msg;
    msg.linear.x = 0.1;
    pub.publish(msg);
    ros::Duration(2.25).sleep();
    // expected: 22.5 cm
    // got: 23cm
*/
/*
    ROS_INFO("move base backward");
    geometry_msgs::Twist msg;
    msg.linear.x = -0.1;
    pub.publish(msg);
    ros::Duration(2).sleep();
    // expected: 20 cm
    // got: 21,5 cm
*/
/*
    ROS_INFO("move base leftwards");
    geometry_msgs::Twist msg;
    msg.linear.y = 0.1;
    pub.publish(msg);
    ros::Duration(2).sleep();
    // expected: 20 cm
    // got: 20cm
*/
/*
    ROS_INFO("move base rightwards");
    geometry_msgs::Twist msg;
    msg.linear.y = -0.1;
    pub.publish(msg);
    ros::Duration(2).sleep();
    // expected: 20 cm
    // got: 19,5cm
*/
/*
    ROS_INFO("turn base left");
    geometry_msgs::Twist msg;
    msg.angular.z = 0.1;
    pub.publish(msg);
    ros::Duration(2).sleep();
*/
/*
    ROS_INFO("turn base right");
    geometry_msgs::Twist msg;
    msg.angular.z = -0.5;
    pub.publish(msg);
    ros::Duration(6.28).sleep();
    //slightly to far
    //expected 180° turn around
*/
    ROS_INFO("move base forward/rightwards");
    geometry_msgs::Twist msg;
    msg.linear.x = 0.05;
    msg.linear.y = -0.05;
    pub.publish(msg);
    ros::Duration(2).sleep();
    // expected: 20 cm
    // got: 19,5cm

    ROS_INFO("stopping");
    msg.linear.x = 0;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = 0;
    pub.publish(msg);

    ros::shutdown();
    return 0;
}
