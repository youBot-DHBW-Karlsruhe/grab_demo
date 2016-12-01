#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "grab_demo/DemoYoubot.h"

int main(int argc, char **argv) {
    ROS_INFO("Initializing ros node and publisher");
    ros::init(argc, argv, "move_base_test_node");
    ros::NodeHandle node;
/*
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
/*
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
*/

    youbot_grab_demo::DemoYoubot demo(youbot_grab_demo::DemoYoubot::DEFAULT_POINT_SECONDS);
    if(!demo.initialize(node)) {
        ROS_ERROR("Could not initialize youbot demo");
        return 1;
    }

    ROS_INFO("Testing move methods");
    demo.moveBase(new youbot_grab_demo::Direction(youbot_grab_demo::FORWARD), 0.2);
    ros::Duration(1).sleep();
    demo.moveBase(new youbot_grab_demo::Direction(youbot_grab_demo::BACKWARD), 0.2);
    ros::Duration(1).sleep();
    demo.moveBase(new youbot_grab_demo::Direction(youbot_grab_demo::LEFT), 0.2);
    ros::Duration(1).sleep();
    demo.moveBase(new youbot_grab_demo::Direction(youbot_grab_demo::RIGHT), 0.2);
    ros::Duration(1).sleep();
    demo.moveBase(new youbot_grab_demo::Direction(youbot_grab_demo::LEFT_FORWARD), 0.2);
    ros::Duration(1).sleep();
    demo.moveBase(new youbot_grab_demo::Direction(youbot_grab_demo::RIGHT_BACKWARD), 0.2);
    ros::Duration(1).sleep();
    demo.moveBase(new youbot_grab_demo::Direction(youbot_grab_demo::RIGHT_FORWARD), 0.2);
    ros::Duration(1).sleep();
    demo.moveBase(new youbot_grab_demo::Direction(youbot_grab_demo::LEFT_BACKWARD), 0.2);
    ros::Duration(1).sleep();

    ROS_INFO("Testing turn methods");
    demo.turnBaseDeg(90.0);
    ros::Duration(1).sleep();
    demo.turnBaseDeg(-90.0);
    ros::Duration(1).sleep();

    if(!demo.returnToInitPose()) {
        ROS_ERROR("main(): failed");
        return 1;
    }

    ros::shutdown();
    return 0;
}
