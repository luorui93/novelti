#include "ros/ros.h"

#define TRACK_MAP 1
#include "fast_dist.cpp"
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>
#include <lthmi_nav/fast_dist.h>
#include <lthmi_nav/map_ros.cpp>
#include <lthmi_nav/IntMap.h>
#include <lthmi_nav/map.h>
#include <lthmi_nav/Experiment.h>


using namespace lthmi_nav;

class RobotModel {
public:
    enum RobotState { WAITING, STOPPED, MOVING };    
    
    ros::NodeHandle* n_;
    ros::Publisher pub_;
    ros::Subscriber sub_opt_pose_;
    ros::Subscriber sub_ident_pose_;
    ros::Subscriber sub_scene_;
    
    geometry_msgs::PoseStamped current_pose_;
    ros::ServiceServer stop_service_;
    MapRos<lthmi_nav::IntMap, int> *my_map_, *track_map_;
    
    vector<Point2D> path_points_;
    double resolution_;
    ros::Time time_started_;
    double max_lin_velocity_, max_ang_velocity_, pub_period_;

    RobotState state_;
    
    RobotModel() {
        n_ = new ros::NodeHandle("~");
        n_->param("max_lin_velocity", max_lin_velocity_, 3.0);
        n_->param("max_ang_velocity", max_ang_velocity_, 3.0);
        n_->param("pub_period", pub_period_, 0.025);
        state_ = WAITING;
        current_pose_.header.frame_id = "/map";
        pub_ = n_->advertise<geometry_msgs::PoseStamped>("/current_pose", 1);
        sub_scene_    = n_->subscribe("/scene", 1, &RobotModel::sceneCallback, this);
        stop_service_ = n_->advertiseService("stop",  &RobotModel::stop, this);
    }
    
    
    void sceneCallback(lthmi_nav::Experiment msg) {
        my_map_ = new MapRos<lthmi_nav::IntMap, int>(msg.mapa);
        resolution_ = msg.mapa.info.resolution;
        
        lthmi_nav::IntMap track_map_msg;
        track_map_msg.info.width = my_map_->width();
        track_map_msg.info.height = my_map_->height();
        track_map_msg.info.resolution = my_map_->resolution();
        track_map_msg.data = vector<int>(my_map_->width()* my_map_->height(), -1);
        track_map_= new MapRos<lthmi_nav::IntMap, int>(track_map_msg);
        
        current_pose_.pose  = msg.init_pose;
        sub_opt_pose_   = n_->subscribe("/optimal_pose", 1, &RobotModel::optimalPoseCB, this);
        sub_ident_pose_ = n_->subscribe("/identified_pose", 1, &RobotModel::optimalPoseCB, this);
        state_ = STOPPED;
        ROS_INFO("robot_model: got scene");
    }
    
    Point2D pose2xy(geometry_msgs::PoseStamped pose) {
        return {
            (int) floor( pose.pose.position.x / resolution_), 
            (int) floor( pose.pose.position.y / resolution_)
        };
    }
    
    void optimalPoseCB(geometry_msgs::PoseStamped optimal_pose) {
        ROS_INFO("robot_model: recieved optimal_pose or identified_pose: (%f,%f)", optimal_pose.pose.position.x,optimal_pose.pose.position.y);
        time_started_ = ros::Time::now();
        
        Point2D src = pose2xy(current_pose_);
        Galaxy glx = calculate_distances(*my_map_, src.x, src.y, *track_map_);
        
        path_points_.clear();
        Point2D p = pose2xy(optimal_pose);
        path_points_.push_back(p);
        int track_star_index;
        do {
            track_star_index = track_map_->get(p.x,p.y);
            p = glx.track_stars[track_star_index];
            path_points_.push_back(p);      //ROS_INFO("robot_model: added: %d,%d", p.x, p.y);
        } while(track_star_index!=0);
        my_map_->clean_dist();
        state_ = MOVING;        
    }
    
    void updateCurrentPose(ros::Time t) {
        double T = (t-time_started_).toSec();
        double dt_seg, gamma;
        Point2D cur,nxt;
        state_ = STOPPED;
        for (int k=path_points_.size()-1; k>=1; k-- ) {
            cur = path_points_[k];
            nxt = path_points_[k-1];
            dt_seg = sqrt( (nxt.x-cur.x)*(nxt.x-cur.x) + (nxt.y-cur.y)*(nxt.y-cur.y) )*resolution_/max_lin_velocity_;
            if (T <= dt_seg) {
                gamma = T/dt_seg;
                state_ = MOVING;
                break;
            }
            T -= dt_seg;
        }
        if (state_ == STOPPED)
            gamma = 1.0;
        current_pose_.pose.position.x = resolution_ * (0.5 + cur.x + (nxt.x-cur.x)*gamma);
        current_pose_.pose.position.y = resolution_ * (0.5 + cur.y + (nxt.y-cur.y)*gamma);
    }


    
    bool stop(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
        sub_opt_pose_.shutdown();
        sub_ident_pose_.shutdown();
        state_ = WAITING;
        ROS_INFO("robot_model: stopped");
        return true;
    }
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "robot_model");
    RobotModel rm;
    ros::Rate r(1.0/rm.pub_period_);
    while (ros::ok()) {
        if (rm.state_ != RobotModel::WAITING) {
            if (rm.state_ == RobotModel::MOVING) 
               rm.updateCurrentPose(ros::Time::now());
            rm.current_pose_.header.stamp = ros::Time::now();
            rm.pub_.publish(rm.current_pose_);
            //ROS_INFO("robot_model: published currrent_pose");
        }
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
