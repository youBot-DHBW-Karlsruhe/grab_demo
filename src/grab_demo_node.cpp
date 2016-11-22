//
// Simple demo program for showing grabbing and releasing an object with the
// KUKA youBot manipulator.
//

#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "brics_actuator/JointPositions.h"

// degree to radiant conversion
#define DEG_TO_RAD(x) ((x) * M_PI / 180.0)

namespace youbot_grab_demo 
{

class DemoApplication {
	public:
        // constant members
        static const int DOF = 5;

        // constructor
        DemoApplication(){};
        // destructor
        ~DemoApplication() {
            delete actionClient;
        };

		/**
         * Initializes the demo application.
         */
        bool initialize(ros::NodeHandle node);

        /**
         * Creates a trajectory message compliant to the youBot follow_joint_trajectory topic.
         */
        control_msgs::FollowJointTrajectoryGoal createTrajectoryGoal(int nPoints, const double jointAngles[][DOF], double pointSeconds);

		/**
         * Creates a message compliant to the youBot gripper_controller/position_command topic.
         */
        brics_actuator::JointPositions createGripperPositionMsg(double left, double right);

        /**
         * Sends a trajectory message to the youBot arm.
         */
        bool sendTrajectoryGoalAndWaitForResponse(const std::string frame, const control_msgs::FollowJointTrajectoryGoal goal);

        /**
         * Sends a position message to the gripper on arm_1.
         */
        void sendGripperPositionMsg(const brics_actuator::JointPositions msg);

    private:
        // member
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *actionClient;
        ros::Publisher pub;

        // disable copy-constructor and assignment operator
        DemoApplication(const DemoApplication&);
        void operator=(const DemoApplication&);

} application_name;

bool DemoApplication::initialize(ros::NodeHandle node) {
    actionClient = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("arm_1/arm_controller/follow_joint_trajectory", true);
    // wait for action client to start
    ROS_INFO("Waiting 10 seconds for action server to start.");
    actionClient->waitForServer(ros::Duration(10));
    std::stringstream logMessage;
	
	// create publisher
	ros::Publisher publisher = node.advertise<brics_actuator::JointPositions>("/arm_1/gripper_controller/position_command", 5);

    // check status of members
	bool initialized = true;
    if(!publisher) {
		initialized = false;
	}

    if(!actionClient->isServerConnected()) {
        initialized = false;
    }
    pub = publisher;
    return initialized;
}

control_msgs::FollowJointTrajectoryGoal DemoApplication::createTrajectoryGoal(const int nPoints, const double jointAngles[][DOF], double pointSeconds) {
    control_msgs::FollowJointTrajectoryGoal msg;

    // set values for all points of trajectory
    for (int p = 0; p < nPoints; p++) { // iterate over all points
        trajectory_msgs::JointTrajectoryPoint point;
        for (int i = 0; i < DOF; i++) {
            point.positions.push_back(jointAngles[p][i]);
            point.velocities.push_back(0);
            point.accelerations.push_back(0);
        }
        point.time_from_start = ros::Duration(pointSeconds * (p+1));
        msg.trajectory.points.push_back(point);
    }

    // set joint names
    for (int i = 0; i < 5; i++) {
        std::stringstream jointName;
        jointName << "arm_joint_" << (i + 1);
        msg.trajectory.joint_names.push_back(jointName.str());
    }

    return msg;
}

brics_actuator::JointPositions DemoApplication::createGripperPositionMsg(double left, double right) {
	brics_actuator::JointPositions msg;
	brics_actuator::JointValue leftGripperValue;
	brics_actuator::JointValue rightGripperValue;
	
	leftGripperValue.joint_uri = "gripper_finger_joint_l";
	leftGripperValue.unit = "m";
	leftGripperValue.value = left;

	rightGripperValue.joint_uri = "gripper_finger_joint_r";
	rightGripperValue.unit = "m";
	rightGripperValue.value = right;

	msg.positions.push_back(leftGripperValue);
	msg.positions.push_back(rightGripperValue);
	return msg;
}

bool DemoApplication::sendTrajectoryGoalAndWaitForResponse(const std::string frame, const control_msgs::FollowJointTrajectoryGoal goal) {
    control_msgs::FollowJointTrajectoryGoal trajectoryGoalMsg(goal);

    ROS_INFO("Sending goal to action server and waiting for result...");

    // fill message header and sent it out
    trajectoryGoalMsg.trajectory.header.frame_id = frame;
    trajectoryGoalMsg.trajectory.header.stamp = ros::Time::now();
    actionClient->sendGoal(trajectoryGoalMsg);

    // wait for reply that action was completed (or cancel after 10 sec)
    actionClient->waitForResult(ros::Duration(20));

    if(actionClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("...Goal executed successfully");
        return true;
    }
    ROS_WARN("...timeout reached, no result received!");
    return false;
}

void DemoApplication::sendGripperPositionMsg(const brics_actuator::JointPositions msg) {
    pub.publish(msg);
	//ros::spinOnce();
}

}

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

