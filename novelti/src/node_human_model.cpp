/*
     subs                                       pubs
                    +--------------------+
/pose_intended ---> |                    | ---> /cmd_intended
                    |  node_human_model  | 
  /map_divided ---> |                    | 
                    +--------------------+
                               |
                               +-->  srv: new_goal (whenever /pose_intended received)
                                        req:  -
                                        resp: -
*/

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>

#include <novelti/StartExperiment.h>
#include <novelti/IntMap.h>
#include <novelti/Command.h>
#include <novelti/common.cpp>

#include <mutex>


namespace novelti {

class SimpleHumanModel : public SynchronizableNode {
public:
    ros::ServiceClient new_goal_client_;
    
    ros::Publisher  pub_cmd_intended_;
    ros::Subscriber sub_map_divided_;
    ros::Subscriber sub_pose_intended_;
    
    int intended_x_, intended_y_;
    std::mutex intended_lock_;

    Command cmd_intended_;
    
    SimpleHumanModel() :
        SynchronizableNode()
    {
        const std::string srv_name = "/novelti_shared_control/new_goal";
        ros::service::waitForService(srv_name, -1);
        new_goal_client_ = node.serviceClient<std_srvs::Empty>(srv_name);
    }
    
    void start(novelti::StartExperiment::Request& req) {
        cmd_intended_ = Command();
        pub_cmd_intended_  = node.advertise<Command>("/cmd_intended", 1, true); //latched
        sub_map_divided_   = node.subscribe("/map_divided", 1, &SimpleHumanModel::mapDividedCallback, this);
        sub_pose_intended_ = node.subscribe("/pose_intended", 1, &SimpleHumanModel::poseIntendedCallback, this);
    }
    
    void stop() {
        pub_cmd_intended_.shutdown();
        sub_map_divided_.shutdown();
        sub_pose_intended_.shutdown();
    }
    
    void mapDividedCallback(IntMapConstPtr msg) {
        int region = msg->data[intended_x_ + intended_y_*msg->info.width];
        cmd_intended_.header.stamp = ros::Time::now();
        cmd_intended_.cmd = region;
        pub_cmd_intended_.publish(cmd_intended_);
        ROS_INFO("%s: /map_divided recieived (SEQ=%d), published /cmd_intended=%d (SEQ=%d).", getName().c_str(), msg->header.seq, region, cmd_intended_.header.seq);
    }
    
    void poseIntendedCallback(geometry_msgs::PoseStampedConstPtr msg) {
        intended_lock_.lock();
            updateVertex(msg->pose, intended_x_, intended_y_);
        intended_lock_.unlock();

        std_srvs::Empty srv;
        if (!ros::service::exists(new_goal_client_.getService(), true)) {
            ROS_ERROR("%s not available anymore",new_goal_client_.getService().c_str());
        }
        if (!new_goal_client_.call(srv)) {
            ROS_ERROR("[Human Model] Service call to %s FAILED ", new_goal_client_.getService().c_str());
            ros::shutdown();
        }
        ROS_INFO("%s: /pose_intended received, vertex=(%d,%d), pose=(%f,%f).", getName().c_str(), intended_x_, intended_y_, msg->pose.position.x, msg->pose.position.y);
    }
};
} //namespace novelti



using namespace novelti;

int main(int argc, char **argv) {
    ros::init(argc, argv, "human_model");
    SimpleHumanModel hm;
    hm.run();
    return 0;
}
