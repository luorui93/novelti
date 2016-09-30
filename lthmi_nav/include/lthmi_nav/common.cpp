#include <math.h> 
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

using namespace ros::this_node;

namespace lthmi_nav {
    
class Vertex {
public:
    int x;
    int y;
    
    Vertex() {
        x=0;
        y=0;
    }
    
    Vertex(geometry_msgs::PoseStamped& pose, double resolution) {
        x = (int) floor( pose.pose.position.x / resolution);
        y = (int) floor( pose.pose.position.y / resolution);
    }

    geometry_msgs::PoseStamped toPose(double resolution) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = x*resolution;
        pose.pose.position.y = y*resolution;
        return pose;
    }
};



#include <lthmi_nav/StartExperiment.h>

class SynchronizableNode {
public:
    ros::NodeHandle node;
    ros::ServiceServer start_service;
    bool neverStarted;
    
    SynchronizableNode() :
        node("~"),
        neverStarted(true),
        start_service(node.advertiseService("start", &SynchronizableNode::srvStart, this))
    {
        //neverStarted = true;
        //start_service  = node.advertiseService("start", srvStart, this);
    }
    
    bool srvStart(lthmi_nav::StartExperiment::Request& req, lthmi_nav::StartExperiment::Response& resp) {
        if (!neverStarted) {
            stop();
            ROS_INFO_NAMED(getNamespace(), "stopped Experiment");
        }
        neverStarted = false;
        start(req);
        ROS_INFO_NAMED(getNamespace(), "started a new Experiment");
        return true;
    }
    
    int run1() {
        ros::spin();
        return 0;
    }
    
    virtual void start(lthmi_nav::StartExperiment::Request& req) = 0;
    virtual void stop()  = 0;
};

// Point pose2point(geometry_msgs::PoseStamped& pose, float resolution) {
//     return {
//         (int) floor( pose->pose.position.x / resolution), 
//         (int) floor( pose->pose.position.y / resolution)
//     };
// }
// 
// geometry_msgs::PoseStamped point2pose(Point2D& point, float resolution) {
//     geometry_msgs::PoseStamped pose;
//     pose.pose.position.x = point.x*resolution;
//     pose.pose.position.y = point.y*resolution;
//     return pose;
// }
}