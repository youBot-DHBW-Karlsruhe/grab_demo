//
// Simple demo program for showing grabbing and releasing an object with the
// KUKA youBot manipulator.
//

#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "brics_actuator/JointPositions.h"
#include "grab_demo/DemoApplication.h"

// degree to radiant conversion
#define DEG_TO_RAD(x) ((x) * M_PI / 180.0)


void prepareForGrab(youbot_grab_demo::DemoApplication *application) {
    ROS_INFO("main(): creating demo trajectory points for preparing a grab");
    const int n= 1;
    double points[n][5] = {
        // start pose:
        // {0.00004027682889217683, -0.0, -0.00010995574287564276, 0.0008185840012874813, 0.0023451325442290006},
        {2.994239875087764, 1.1520884828390492, -2.700355965393107, 2.000221069091924, 2.9442475376037303}};

    // create trajectory goal message
    control_msgs::FollowJointTrajectoryGoal goalMsg = application->createTrajectoryGoal(n, points, 6);

    bool goalReached2 = application->sendTrajectoryGoalAndWaitForResponse("arm_link_0", goalMsg);
    if(goalReached2) {
        ROS_INFO("Goal was successfully sent and executed.");
    }

    brics_actuator::JointPositions gripperPositionMsg = application->createGripperPositionMsg(0.0115, 0.0115);
    application->sendGripperPositionMsg(gripperPositionMsg);
}

void grab(youbot_grab_demo::DemoApplication *application) {
    ROS_INFO("main(): creating demo trajectory points for grab");
    const int n= 1;
    double points[n][5] = {
        {2.953187717239413, 2.4635926544333344, -1.7269394542799927, 2.8039599388965972, 2.933296211100019}};

    // create trajectory goal message
    control_msgs::FollowJointTrajectoryGoal goalMsg = application->createTrajectoryGoal(n, points, 6);

    bool goalReached2 = application->sendTrajectoryGoalAndWaitForResponse("arm_link_0", goalMsg);
    if(goalReached2) {
        ROS_INFO("Goal was successfully sent and executed.");
    }

    brics_actuator::JointPositions gripperPositionMsg = application->createGripperPositionMsg(0.006, 0.006);
    application->sendGripperPositionMsg(gripperPositionMsg);
}

void drop(youbot_grab_demo::DemoApplication *application) {
    ROS_INFO("main(): creating demo trajectory points for drop on metal plate");
    const int n= 2;
    double points[n][5] = {
        {2.994239875087764, 1.1520884828390492, -2.700355965393107, 2.000221069091924, 2.9442475376037303},
        {2.94689446272501, 0.08719933455156284, -2.822768123140233, 0.053185836191759595, 5.855950830183301}
    };

    // create trajectory goal message
    control_msgs::FollowJointTrajectoryGoal goalMsg = application->createTrajectoryGoal(n, points, 6);

    bool goalReached2 = application->sendTrajectoryGoalAndWaitForResponse("arm_link_0", goalMsg);
    if(goalReached2) {
        ROS_INFO("Goal was successfully sent and executed.");
    }

    brics_actuator::JointPositions gripperPositionMsg = application->createGripperPositionMsg(0.0115, 0.0115);
    application->sendGripperPositionMsg(gripperPositionMsg);
}

void returnToTower(youbot_grab_demo::DemoApplication *application) {
    ROS_INFO("main(): creating demo trajectory points for tower");
    const int n= 1;
    double points[n][5] = {
        {2.994239875087764, 1.1520884828390492, -2.700355965393107, 2.000221069091924, 2.9442475376037303}
    };

    // create trajectory goal message
    control_msgs::FollowJointTrajectoryGoal goalMsg = application->createTrajectoryGoal(n, points, 6);

    bool goalReached2 = application->sendTrajectoryGoalAndWaitForResponse("arm_link_0", goalMsg);
    if(goalReached2) {
        ROS_INFO("Goal was successfully sent and executed.");
    }
}

int main(int argc, char **argv) {
    // initialize ros node
    ROS_INFO("main(): calling ros::init and initializing node");
    ros::init(argc, argv, "grab_demo_node");
    ros::NodeHandle node;

    // create demo application and run it
    ROS_INFO("main(): creating demo application and initializing it");
    youbot_grab_demo::DemoApplication demo;
    bool successfullyInitialized = demo.initialize(node);
    if(!successfullyInitialized) {
        ROS_ERROR("Could not initialize demo application. It may not be started yet?");
        return 1;
    }

    ROS_INFO("Preparing for grasping...");
    prepareForGrab(&demo);

    ROS_INFO("Grasping object...");
    grab(&demo);

    ROS_INFO("Dropping object...");
    drop(&demo);

    ROS_INFO("Returning to initial position...");
    returnToTower(&demo);

    /*
    ROS_INFO("main(): creating demo trajectory points and sending goals");
	// tower position goal
    const int n= 5;
    double points[n][5] = {
		// start pose:
		// {0.00004027682889217683, -0.0, -0.00010995574287564276, 0.0008185840012874813, 0.0023451325442290006},
        {2.994239875087764, 1.1520884828390492, -2.700355965393107, 2.000221069091924, 2.9442475376037303},
		{2.953187717239413, 2.4635926544333344, -1.7369394542799927, 2.8039599388965972, 2.933296211100019},
		{2.994239875087764, 1.1520884828390492, -2.700355965393107, 2.000221069091924, 2.9442475376037303},
		{2.94689446272501, 0.08719933455156284, -2.822768123140233, 0.053185836191759595, 5.855950830183301},
		{2.994239875087764, 1.1520884828390492, -2.700355965393107, 2.000221069091924, 2.9442475376037303} };

	// create trajectory goal message
    control_msgs::FollowJointTrajectoryGoal goalMsg = demo.createTrajectoryGoal(n, points, 6);

	bool goalReached2 = demo.sendTrajectoryGoalAndWaitForResponse("arm_link_0", goalMsg);
    if(goalReached2) {
        ROS_INFO("Goal was successfully sent and executed.");
    }*/
	
	return 0;
}

