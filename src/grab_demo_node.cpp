//
// Simple demo program for showing grabbing and releasing an object with the
// KUKA youBot manipulator.
//

#include "ros/ros.h"
#include "grab_demo/DemoYoubot.h"

int main(int argc, char **argv) {
    // initialize ros node
    ROS_INFO("main(): calling ros::init and initializing node");
    ros::init(argc, argv, "grab_demo_node");
    ros::NodeHandle node;

    // create demo application and run it
    ROS_INFO("main(): creating demo application and initializing it");
    double timeForEachPointInSeconds = 5.5;
    youbot_grab_demo::DemoYoubot demo(timeForEachPointInSeconds);
    //youbot_grab_demo::DemoYoubot demo(youbot_grab_demo::DemoYoubot::DEFAULT_POINT_SECONDS);
    bool successfullyInitialized = demo.initialize(node);
    if(!successfullyInitialized) {
        ROS_ERROR("Could not initialize demo application. It may not be started yet?");
        return 1;
    }


    ROS_INFO("main(): moving to grab position");
    demo.moveBase(new youbot_grab_demo::Direction(youbot_grab_demo::FORWARD), 0.6);
    ros::Duration(0.2).sleep();
    demo.turnBaseDeg(90.0);
    ros::Duration(0.2).sleep();
    demo.moveBase(new youbot_grab_demo::Direction(youbot_grab_demo::RIGHT), 0.43);
    ros::Duration(0.2).sleep();
    demo.moveBase(new youbot_grab_demo::Direction(youbot_grab_demo::FORWARD), 0.09);
    ros::Duration(0.2).sleep();


    ROS_INFO("main(): grasping object");
    if(!demo.grab()) {
        ROS_ERROR("main(): grasping object failed");
        return 1;
    }

    ROS_INFO("main(): dropping object");
    if(!demo.drop()) {
        ROS_ERROR("main(): dropping object failed");
        return 1;
    }

    ROS_INFO("main(): returning arm to initial pose");
    if(!demo.returnToInitPose()) {
        ROS_ERROR("main(): failed");
        return 1;
    }
	
	return 0;
}

