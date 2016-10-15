#include <math.h> 
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

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
    
    Vertex(const geometry_msgs::Pose& pose, double resolution) {
        x = (int) round( pose.position.x / resolution);
        y = (int) round( pose.position.y / resolution);
    }

    geometry_msgs::PoseStamped toPose(double resolution) {
        return toPose(x,y,resolution);
    }
    
    static geometry_msgs::PoseStamped toPose(int x, int y, double resolution) {
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
            ROS_INFO("%s: stopped Experiment", getName().c_str());
        }
        neverStarted = false;
        start(req);
        ROS_INFO("%s: started a new Experiment. Init pose=(%f,%f)", getName().c_str(), req.init_pose.position.x, req.init_pose.position.y);
        return true;
    }
    
    int run() {
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