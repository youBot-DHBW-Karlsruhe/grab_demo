//
// Simple demo program for showing grabbing and releasing an object with the
// KUKA youBot manipulator.
//

#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "control_msgs/FollowJointTrajectoryAction.h"

// degree to radiant conversion
#define DEG_TO_RAD(x) ((x) * M_PI / 180.0)

namespace youbot_grab_demo 
{

class DemoApplication {
	public:
        // constant members
        static const int DOF = 5;

        // constructor
        DemoApplication(){}
        // destructor
        ~DemoApplication() {
            delete actionClient;
        };

		/**
         * Initializes the demo application.
         */
        bool initialize();

        /**
         * Creates a trajectory message compliant to the youBot follow_joint_trajectory topic.
         */
        control_msgs::FollowJointTrajectoryGoal createTrajectoryGoal(int nPoints, const double jointAngles[][DOF], double pointSeconds);

        /**
         * Sends a trajectory message to the youBot arm.
         */
        bool sendTrajectoryGoalAndWaitForResponse(const std::string frame, const control_msgs::FollowJointTrajectoryGoal goal);

    private:
        // member
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *actionClient;

        // disable copy-constructor and assignment operator
        DemoApplication(const DemoApplication&);
        void operator=(const DemoApplication&);

} application_name;

bool DemoApplication::initialize() {
    actionClient = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("arm_1/arm_controller/follow_joint_trajectory", true);
    // wait for action client to start
    ROS_INFO("Waiting 10 seconds for action server to start.");
    actionClient->waitForServer(ros::Duration(10));
    std::stringstream logMessage;
    //logMessage << "action client state ist: " << actionClient->getState().getText();
    //ROS_INFO(logMessage.get());

    if(actionClient->isServerConnected()) {
        return true;
    }
    return false;
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

}

int main(int argc, char **argv) {
    // initialize ros node
    ROS_INFO("main(): calling ros::init and initializing node");
    ros::init(argc, argv, "arm_position_observe");
    ros::NodeHandle node;

    // create demo application and run it
    ROS_INFO("main(): creating demo application and initializing it");
    youbot_grab_demo::DemoApplication demo;
    bool successfullyInitialized = demo.initialize();
    if(!successfullyInitialized) {
        ROS_ERROR("Could not initialize action client. It may not be started yet?");
        return 1;
    }

    ROS_INFO("main(): creating demo trajectory points and sending goals");
	// tower position goal
    const int n= 1;
    double points[n][5] = {
        2.9551310742334604, 0.14250948982774467, -0.4021081516962256, 2.768384720678482, 2.9548006340527606
    };

	// create trajectory goal message
    control_msgs::FollowJointTrajectoryGoal goalMsg = demo.createTrajectoryGoal(n, points, 6);

	bool goalReached2 = demo.sendTrajectoryGoalAndWaitForResponse("arm_link_0", goalMsg);
    if(goalReached2) {
        ROS_INFO("Goal was successfully sent and executed.");
    }
	
	return 0;
}

