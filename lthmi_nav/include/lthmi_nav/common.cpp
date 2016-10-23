#include <math.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

#include <lthmi_nav/StartExperiment.h>
#include <std_srvs/Empty.h>

using namespace ros::this_node;

namespace lthmi_nav {
    
/*class Vertex {
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

//     geometry_msgs::PoseStamped toPose(double resolution) {
//         return toPose(x,y,resolution);
//     }
//     
//     static geometry_msgs::Pose toPose(int x, int y, double resolution) {
//         geometry_msgs::PoseStamped pose;
//         pose.position.x = x*resolution;
//         pose.position.y = y*resolution;
//         pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, -M_PI/2, 0.0);
//         return pose;
//     }
    
    static void updPose(geometry_msgs::PoseStamped& pose, int x, int y, double resolution) {
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = x*resolution;
        pose.pose.position.y = y*resolution;
        pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, -M_PI/2, 0.0);
    }
};*/





class SynchronizableNode {
public:
    ros::NodeHandle node;
    ros::ServiceServer start_service;
    ros::ServiceServer stop_service;
    double resolution;
    
    SynchronizableNode() :
        node("~"),
        start_service(node.advertiseService("start", &SynchronizableNode::srvStart, this)),
        stop_service(node.advertiseService("stop", &SynchronizableNode::srvStop, this))
    {}
    
    bool srvStart(lthmi_nav::StartExperiment::Request& req, lthmi_nav::StartExperiment::Response& resp) {
        resolution = req.map.info.resolution;
        start(req);
        ROS_INFO("%s: started a new Experiment. Init pose=(%f,%f)", getName().c_str(), req.init_pose.position.x, req.init_pose.position.y);
        return true;
    }
    
    bool srvStop(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp) {
        stop();
        ROS_INFO("%s: stopped Experiment", getName().c_str());
        return true;
    }
    
    int run() {
        ros::spin();
        return 0;
    }
    
    void updateVertex(const geometry_msgs::Pose& pose, int& x, int& y) {
        x = (int) round( pose.position.x / resolution);
        y = (int) round( pose.position.y / resolution);
    }
    
    void updatePose(geometry_msgs::PoseStamped& pose, int x, int y) {
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = x*resolution;
        pose.pose.position.y = y*resolution;
        pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, -M_PI/2, 0.0);
    }
    
    
    virtual void start(lthmi_nav::StartExperiment::Request& req) = 0;
    virtual void stop()  = 0;
};

}