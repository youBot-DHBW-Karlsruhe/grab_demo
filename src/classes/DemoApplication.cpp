#include "grab_demo/DemoApplication.h"

namespace youbot_grab_demo {

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
