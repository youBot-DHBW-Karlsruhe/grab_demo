//
// Simple demo program for showing grabbing and releasing an object with the
// KUKA youBot manipulator.
//

#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "brics_actuator/JointPositions.h"
#include "grab_demo/DemoYoubot.h"

// degree to radiant conversion
#define DEG_TO_RAD(x) ((x) * M_PI / 180.0)

int main(int argc, char **argv) {
    // initialize ros node
    ROS_INFO("main(): calling ros::init and initializing node");
    ros::init(argc, argv, "grab_demo_node");
    ros::NodeHandle node;

    // create demo application and run it
    ROS_INFO("main(): creating demo application and initializing it");
    youbot_grab_demo::DemoYoubot demo(youbot_grab_demo::DemoYoubot::DEFAULT_POINT_SECONDS);
    bool successfullyInitialized = demo.initialize(node);
    if(!successfullyInitialized) {
        ROS_ERROR("Could not initialize demo application. It may not be started yet?");
        return 1;
    }

    ROS_INFO("Grasping object...");
    demo.grab();

    ROS_INFO("Dropping object...");
    demo.drop();
	
	return 0;
}

