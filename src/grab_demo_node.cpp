//
// Simple demo program for showing grabbing and releasing an object with the
// KUKA youBot manipulator.
//

#include "ros/ros.h"
#include "grab_demo/DemoYoubot.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PolygonStamped.h"
#include "object_finder_2d/NearestPoint.h"

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
const double WAIT_TIME = 1.0;
const double TIME_FOR_EACH_POINT_IN_SECONDS = 5.5;
const int COUNTER_THRESHOLD = 5;
const double NEAR  = 0.25;
const double FAR   = 0.35;
const double LEFT  = 0.05;
const double RIGHT =-0.05;

bool insideBoundings(const geometry_msgs::Point32 point) {
    int x = point.x;
    int y = point.y;
    int z = point.z;

    if(x <= FAR   && x >= NEAR &&
       y <= RIGHT && y >= LEFT) {
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

    /*
    // create demo application and initialize it
    ROS_INFO("main(): creating demo application and initializing it");
    youbot_grab_demo::DemoYoubot demo(TIME_FOR_EACH_POINT_IN_SECONDS);
    //youbot_grab_demo::DemoYoubot demo(youbot_grab_demo::DemoYoubot::DEFAULT_POINT_SECONDS);
    bool successfullyInitialized = demo.initialize(node);
    if(!successfullyInitialized) {
        ROS_ERROR("Could not initialize demo application. It may not be started yet?");
        return 1;
    }

    */
    // initialize boundary publisher
    ros::Publisher pubBoundaries = node.advertise<geometry_msgs::PolygonStamped>("/boundaries", 10);
    geometry_msgs::PolygonStamped polyMsg = initializeBoundaryMsg();

    /*
    // initialize NearestPointService Listener
    NearestPointListener npService(node);
    */
    int counter;
    reset(&counter);

    // main loop:
    while(ros::ok()) {
        // wait a moment and start / test again
        ros::Duration(WAIT_TIME/2.).sleep();
        ros::spinOnce();
        ros::Duration(WAIT_TIME/2.).sleep();
        if(!ros::ok()) return 1;

        ROS_INFO("main(): Testing if object is inside boundaries");
        publishBoundaries(pubBoundaries, polyMsg);
        geometry_msgs::Point32 nearest; // = npService.nearestPoint();

        // counter to be sure there is an object
        if(insideBoundings(nearest)) {
            counter++;
        } else {
            ROS_INFO_STREAM("main(): counter reset, was " << counter);
            reset(&counter);
        }

        // if we found enough points inside the boundaries grab the object
        if(counter >= COUNTER_THRESHOLD) {
            ROS_INFO("main(): grasping object");
            /*
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
            }*/

            // reset counter
            reset(&counter);
        }
    }
	
	return 0;
}

