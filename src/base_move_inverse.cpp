#include "ros/ros.h"
#include "grab_demo/DemoYoubot.h"

int main(int argc, char **argv) {
    ROS_INFO("main(): Initializing ros node and publisher");
    ros::init(argc, argv, "base_move_inverse");
    ros::NodeHandle node;

    youbot_grab_demo::DemoYoubot demo(youbot_grab_demo::DemoYoubot::DEFAULT_POINT_SECONDS);
    if(!demo.initialize(node)) {
        ROS_ERROR("Could not initialize youbot demo");
        return 1;
    }

    ROS_INFO("main(): Traversing inverse demo movement");
    demo.moveBase(new youbot_grab_demo::Direction(youbot_grab_demo::BACKWARD), 0.1);
    ros::Duration(0.2).sleep();
    demo.moveBase(new youbot_grab_demo::Direction(youbot_grab_demo::LEFT), 0.4);
    ros::Duration(0.2).sleep();
    demo.turnBaseDeg(-90.0);
    ros::Duration(0.2).sleep();
    demo.moveBase(new youbot_grab_demo::Direction(youbot_grab_demo::BACKWARD), 0.6);
    ros::Duration(0.2).sleep();



    if(!demo.returnToInitPose()) {
        ROS_ERROR("main(): failed");
        return 1;
    }

    ros::shutdown();
    return 0;
}
