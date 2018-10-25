#ifndef NOVELTI_SYNCHRONIZABLE_NODE_INCLUDE
#define NOVELTI_SYNCHRONIZABLE_NODE_INCLUDE

#include <math.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

#include <novelti/StartExperiment.h>
#include <std_srvs/Empty.h>

using namespace ros::this_node;

namespace novelti {
    

class SynchronizableNode {
    
public:
    SynchronizableNode() :
        node_("~"),
        start_service_(node_.advertiseService("start", &SynchronizableNode::srvStart, this)),
        stop_service_(node_.advertiseService("stop", &SynchronizableNode::srvStop, this))
    {
        node_.param<bool>("check_sync", check_sync_, true);
    }
    
    int run() {
        ros::spin();
        return 0;
    }
    
protected:    
    ros::NodeHandle node_;
    ros::ServiceServer start_service_;
    ros::ServiceServer stop_service_;
    double resolution_;
    bool check_sync_;
    
    bool srvStart(novelti::StartExperiment::Request& req, novelti::StartExperiment::Response& resp) {
        resolution_ = req.map.info.resolution;
        start(req);
        ROS_INFO("%s: started a new Experiment. Init pose=(%f,%f)", getName().c_str(), req.init_pose.position.x, req.init_pose.position.y);
        return true;
    }
    
    bool srvStop(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp) {
        stop();
        ROS_INFO("%s: stopped Experiment", getName().c_str());
        return true;
    }
    
    static void updateVertex(const geometry_msgs::Pose& pose, int& x, int& y, double resol) {
        x = (int) round( pose.position.x / resol);
        y = (int) round( pose.position.y / resol);
    }
    
    static void updatePose(geometry_msgs::PoseStamped& pose, int x, int y, double resol) {
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = x*resol;
        pose.pose.position.y = y*resol;
        pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
    }

    void updateVertex(const geometry_msgs::Pose& pose, int& x, int& y) {
        updateVertex(pose, x , y, resolution_);
    }
    
    void updatePose(geometry_msgs::PoseStamped& pose, int x, int y) {
        updatePose(pose, x, y, resolution_);
    }

    // void updateVertex(const geometry_msgs::Pose& pose, int& x, int& y) {
    //     x = (int) round( pose.position.x / resolution);
    //     y = (int) round( pose.position.y / resolution);
    // }
    
    // void updatePose(geometry_msgs::PoseStamped& pose, int x, int y) {
    //     pose.header.stamp = ros::Time::now();
    //     pose.pose.position.x = x*resolution;
    //     pose.pose.position.y = y*resolution;
    //     pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, -M_PI/2, 0.0);
    // }
    
    
    virtual void start(novelti::StartExperiment::Request& req) = 0;
    virtual void stop()  = 0;
};

}

#endif