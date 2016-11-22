#ifndef GRAB_DEMO_APPLICATION_H
#define GRAB_DEMO_APPLICATION_H

#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "brics_actuator/JointPositions.h"

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
        }

        // methods
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

};

}
#endif // GRAB_DEMO_APPLICATION_H
