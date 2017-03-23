//
// Simple demo program for showing grabbing and releasing an object with the
// KUKA youBot manipulator.
//

#include "ros/ros.h"
#include "grab_demo/DemoYoubot.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PolygonStamped.h"
#include "object_finder_2d/NearestPoint.h"

#define RETURN_OK 0;
#define RETURN_GRAB_ERROR 1;
#define RETURN_DROP_ERROR 2;
#define RETURN_REINIT_ERROR 3;
#define RETURN_TOINIT_ERROR 4;
#define RETURN_INIT_ERROR 10;

class NearestPointListener {
private:
    ros::ServiceClient nearestPointService;
    geometry_msgs::Point32 point;


public:
    NearestPointListener(ros::NodeHandle n) {
        ros::service::waitForService("nearest_point");
        nearestPointService = n.serviceClient<object_finder_2d::NearestPoint>("nearest_point");
    }

    geometry_msgs::Point32 nearestPoint() {
        object_finder_2d::NearestPoint srv;
        srv.request.command = "GET";
        if(nearestPointService.call(srv)) {
            return srv.response.point;
        } else {
            ROS_ERROR("Cleaner: Request to nearest_point service failed");
            return geometry_msgs::Point32();
        }
    }

};

const std::string BASE_FRAME = "base_link";
const double WAIT_TIME = 0.5;
const double TIME_FOR_EACH_POINT_IN_SECONDS = 5.5;
const int COUNTER_THRESHOLD = 4;
const double NEAR  = 0.39;
const double FAR   = 0.47;
const double LEFT  = 0.03;
const double RIGHT =-0.03;

bool insideBoundings(const geometry_msgs::Point32 point) {
    double x = point.x;
    double y = point.y;
    double z = point.z;

    if(x <= FAR   && x >= NEAR &&
       y <= LEFT && y >= RIGHT) {
        return true;
    }
    return false;
}

void reset(int* const pCounter) {
    *pCounter = 0;
}

geometry_msgs::PolygonStamped initializeBoundaryMsg() {
    geometry_msgs::PolygonStamped polyMsg;
    geometry_msgs::Point32 p;
    p.z = 0.0;

    polyMsg.header.frame_id = BASE_FRAME;
    polyMsg.header.stamp = ros::Time::now();
    polyMsg.header.seq = 0;

    // 1.
    p.x = NEAR;
    p.y = LEFT;
    polyMsg.polygon.points.push_back(p);

    // 2.
    p.x = NEAR;
    p.y = RIGHT;
    polyMsg.polygon.points.push_back(p);

    // 3.
    p.x = FAR;
    p.y = RIGHT;
    polyMsg.polygon.points.push_back(p);

    // 4.
    p.x = FAR;
    p.y = LEFT;
    polyMsg.polygon.points.push_back(p);

    return polyMsg;
}

void publishBoundaries(const ros::Publisher& publisher, geometry_msgs::PolygonStamped& msg) {
    msg.header.stamp = ros::Time::now();
    msg.header.seq++;

    publisher.publish(msg);
}

int main(int argc, char **argv) {
    // initialize ros node
    ROS_INFO("main(): calling ros::init and initializing node");
    ros::init(argc, argv, "grab_demo_node");
    ros::NodeHandle node;
    int return_code = RETURN_OK;

    // create demo application and initialize it
    ROS_INFO("main(): creating demo application and initializing it");
    youbot_grab_demo::DemoYoubot demo(TIME_FOR_EACH_POINT_IN_SECONDS);
    //youbot_grab_demo::DemoYoubot demo(youbot_grab_demo::DemoYoubot::DEFAULT_POINT_SECONDS);
    bool successfullyInitialized = demo.initialize(node);
    if(!successfullyInitialized) {
        ROS_ERROR("Could not initialize demo application. It may not be started yet?");
        return RETURN_INIT_ERROR;
    }

    // initialize boundary publisher
    ROS_INFO("main(): initializing boundary publisher");
    ros::Publisher pubBoundaries = node.advertise<geometry_msgs::PolygonStamped>("/boundaries", 10);
    geometry_msgs::PolygonStamped polyMsg = initializeBoundaryMsg();

    // initialize NearestPointService Listener
    ROS_INFO("main(): initializing nearest point listener");
    NearestPointListener npService(node);
    int counter;
    reset(&counter);

    ROS_INFO("main(): Waiting 10 seconds before startup");
    ros::Duration(10.0 - WAIT_TIME).sleep();

    // main loop:
    ROS_INFO("main(): Starting main routine...");
    while(ros::ok()) {
        // wait a moment and start / test again
        ros::Duration(WAIT_TIME/2.).sleep();
        ros::spinOnce();
        ros::Duration(WAIT_TIME/2.).sleep();
        if(!ros::ok()) break;

        publishBoundaries(pubBoundaries, polyMsg);
        geometry_msgs::Point32 nearest = npService.nearestPoint();

        // counter to be sure there is an object
        //ROS_INFO_STREAM("Testing point (" << nearest.x << "/"
        //                                  << nearest.y << "/"
        //                                  << nearest.z << ")");
        if(insideBoundings(nearest)) {
            counter++;
            //ROS_INFO_STREAM("main(): was inside --> new counter: " << counter);
        } else {
            reset(&counter);
            //ROS_INFO_STREAM("main(): was outside --> new counter:"<< counter);
        }

        // if we've found enough points inside the boundaries grab the object
        if(counter >= COUNTER_THRESHOLD) {
            ROS_INFO_STREAM("main(): grasping object after " << WAIT_TIME*4 << "seconds");
            // wait a moment
            ros::Duration(WAIT_TIME*2).sleep();
            ros::spinOnce();
            ros::Duration(WAIT_TIME*2).sleep();
            if(!ros::ok()) break;

            if(!demo.grab()) {
                ROS_ERROR("main(): grasping object failed");
                return_code = RETURN_GRAB_ERROR;
                break;
            }

            ROS_INFO("main(): dropping object");
            if(!demo.drop()) {
                ROS_ERROR("main(): dropping object failed");
                return_code = RETURN_DROP_ERROR;
                break;
            }

            ROS_INFO("main(): returning arm to initial pose");
            double poseObserve[demo.DOF] = ARM_POSE_OBSERVE;
            if(!demo.moveArmToPose(poseObserve)) {
                ROS_ERROR("main(): reinitializing failed");
                return_code = RETURN_REINIT_ERROR;
                break;
            }

            // reset counter
            reset(&counter);
        }
    }

    if(!demo.returnToInitPose()) {
        ROS_ERROR("main(): could not return to init pose");
        return_code = RETURN_TOINIT_ERROR;
    }
	
    return return_code;
}

