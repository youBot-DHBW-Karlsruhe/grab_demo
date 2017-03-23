#ifndef GRAB_DEMO_APPLICATION_H
#define GRAB_DEMO_APPLICATION_H

#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "brics_actuator/JointPositions.h"
#include "geometry_msgs/Twist.h"

namespace youbot_grab_demo
{

/*
#define ARM_POSE_INIT    {0.00004027682889217683, -0.0, -0.00010995574287564276, 0.0008185840012874813, 0.0023451325442290006}
#define ARM_POSE_TOWER   {2.994239875087764, 1.1520884828390492, -2.700355965393107, 2.000221069091924, 2.9442475376037303}
#define ARM_POSE_GRAB    {2.953187717239413, 2.4635926544333344, -1.7269394542799927, 2.8039599388965972, 2.933296211100019}
#define ARM_POSE_DROP    {2.94689446272501, 0.08719933455156284, -2.822768123140233, 0.053185836191759595, 5.855950830183301}
#define ARM_POSE_OBSERVE {2.9551310742334604, 0.14250948982774467, -0.4021081516962256, 2.768384720678482, 2.9548006340527606}
*/

#define ARM_POSE_INIT    {0.00004027682889217683, -0.0, -0.00010995574287564276, 0.0008185840012874813, 0.0023451325442290006}
#define ARM_POSE_TOWER   {2.952, 1.1520884828390492, -2.700355965393107, 2.000221069091924, 2.94}
#define ARM_POSE_GRAB    {2.952, 2.356879196283512, -1.72240958825714, 2.704889150848885, 2.94}
#define ARM_POSE_DROP    {2.952, 0.5046787352261988, -3.409696170647146, 0.49811942672939574, 2.94}
#define ARM_POSE_OBSERVE {2.952, 0.14250948982774467, -0.4021081516962256, 2.768384720678482, 2.94}

// degree to radiant conversion
#define DEG_TO_RAD(x) ((x) * M_PI / 180.0)


enum Type {
    LEFT_FORWARD = 1, FORWARD, RIGHT_FORWARD,
    LEFT, STOP, RIGHT,
    LEFT_BACKWARD, BACKWARD, RIGHT_BACKWARD
};

class Direction {
    public:
        const Type type;

        Direction(Type pType): type(pType){}
        ~Direction(){}

        bool isDiagonal() const;
};

class DemoYoubot {
    public:
        // constant members
        static const int DOF = 5;
        static const double TIMEOUT = 20;
        static const double DEFAULT_POINT_SECONDS = 5.0;

        // constructor
        DemoYoubot(): pointSeconds(DEFAULT_POINT_SECONDS){}

        DemoYoubot(double movementSecondsForEachPoint): pointSeconds(movementSecondsForEachPoint){}

        // destructor
        ~DemoYoubot() {
            delete actionClient;
        }

        // methods
        /**
         * Initializes the youBot.
         */
        bool initialize(ros::NodeHandle node);

        /**
         * Performs an object grab in front of the youBot.
         */
        bool grab();

        /**
         * Drops a grabbed object on the metal plate of the youBot.
         */
        bool drop();

        /**
         * Returns arm and gripper to the initial pose of the youbot driver.
         */
        bool returnToInitPose();

        /**
         * Open gripper -> drop an object.
         */
        void openGripper();

        /**
         * Close gripper -> grab an object.
         */
        void closeGripper();

        /**
         * Moves youBot arm to the specified pose. The pose is in joint space and depends on the DOF member.
         */
        bool moveArmToPose(const double pose[DOF]);

        /**
         * Moves the base in the specified direction.
         */
        void moveBase(const Direction *direction, double distanceInMeters);

        /**
         * Turns the base in z (left). Negative values corresponds to the other direction (right).
         */
        void turnBaseRad(double angleInRad);

        /**
         * Turns the base in z (left). Negative values corresponds to the other direction (right).
         */
        void turnBaseDeg(double angleInDeg);

    private:
        // constants
        const double pointSeconds;
        geometry_msgs::Twist stopMsg;

        // member
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *actionClient;
        ros::Publisher pubArm;
        ros::Publisher pubBase;

        // disable copy-constructor and assignment operator
        DemoYoubot(const DemoYoubot&);
        void operator=(const DemoYoubot&);

        // methods
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

};

}
#endif // GRAB_DEMO_APPLICATION_H
