/*
     subs                                       pubs
                    +--------------------+
                    |                    |
 /pose_desired ---> |                    | ---> /pose_current
                    |  node_robot_model  |
                    |                    | ---> /reach_area   | debug
                    |                    |
                    +--------------------+
                               ^
                               |
                           srv: start
                               req:  map
                                   init_pose
                               resp: -
*/


#include <CompoundMap.h>
#include <CWave2.h>

#include "ros/ros.h"
#include <math.h>
#include <geometry_msgs/PoseStamped.h>

#include <lthmi_nav/StartExperiment.h>
#include <lthmi_nav/common.cpp>

using namespace cwave;

namespace lthmi_nav {

class NoKinRobotModel : public SynchronizableNode {
public:
    enum RobotState { WAITING, STOPPED, MOVING };    
    RobotState state_;
    
    ros::Publisher  pub_pose_current_;
    ros::Subscriber sub_pose_desired_;
    
    geometry_msgs::PoseStamped pose_current_;
    
    vector<Point> path_;
    double resolution_;
    ros::Time time_started_;
    CompoundMap cmap_;
    double max_vel_, max_vel_ang_, pub_period_;
    
    NoKinRobotModel() :
        SynchronizableNode()
    {
        node.param("max_vel", max_vel_,  0.5);
        node.param("max_vel_ang", max_vel_ang_, 0.5); //currently ignored
        node.param("pub_period", pub_period_, 0.025);
        state_ = WAITING;
        pose_current_.header.frame_id = "/map";
    }
    
    void start(lthmi_nav::StartExperiment::Request& req) {
        pose_current_.pose  = req.init_pose;
        resolution_ = req.map.info.resolution;

        new (&cmap_) CompoundMap(req.map.info.width, req.map.info.height);
        for (int x=0; x<req.map.info.width; x++)
            for (int y=0; y<req.map.info.height; y++)
                if (req.map.data[x + y*req.map.info.width]==0)
                    cmap_.setPixel(x,y, FREED); //free
        
        pub_pose_current_ = node.advertise<geometry_msgs::PoseStamped>("/pose_current", 1, false); //not latched
        sub_pose_desired_ = node.subscribe("/pose_desired", 1, &NoKinRobotModel::desiredPoseCallback, this);
        state_ = STOPPED;
    }
    
    void desiredPoseCallback(geometry_msgs::PoseStamped pose_des) {
        ROS_INFO("robot_model: recieved desired pose: (%f,%f)", pose_des.pose.position.x,pose_des.pose.position.y);
        time_started_ = ros::Time::now();
        Vertex src(pose_current_.pose, resolution_);
        CWave2 cw(cmap_);
        CWave2Processor dummy;
        cw.setProcessor(&dummy);
        cw.calc(Point(src.x,src.y));
        
        Vertex dst(pose_des.pose, resolution_);
        Point p(dst.x, dst.y);
        path_.clear();
        path_.push_back(p);
        TrackStar tstar;
        int track_star_id;
        do {
            track_star_id = cmap_.getTrackStarId(p.x,p.y);
            tstar = cmap_.getTrackStar(track_star_id);
            p = Point(tstar.x,tstar.y);
            path_.push_back(p);      //ROS_INFO("robot_model: added: %d,%d", p.x, p.y);
        } while(track_star_id!=0);
        cmap_.clearDist();
        state_ = MOVING;
    }
    
    void updateCurrentPose(ros::Time t) {
        double T = (t-time_started_).toSec();
        double dt_seg, gamma;
        Point cur,nxt;
        state_ = STOPPED;
        for (int k=path_.size()-1; k>=1; k-- ) {
            cur = path_[k];
            nxt = path_[k-1];
            dt_seg = sqrt( (nxt.x-cur.x)*(nxt.x-cur.x) + (nxt.y-cur.y)*(nxt.y-cur.y) )*resolution_/max_vel_;
            if (T <= dt_seg) {
                gamma = T/dt_seg;
                state_ = MOVING;
                break;
            }
            T -= dt_seg;
        }
        if (state_ == STOPPED)
            gamma = 1.0;
        pose_current_.pose.position.x = resolution_ * (cur.x + (nxt.x-cur.x)*gamma);
        pose_current_.pose.position.y = resolution_ * (cur.y + (nxt.y-cur.y)*gamma);
    }

    void stop() {
        sub_pose_desired_.shutdown();
        pub_pose_current_.shutdown();
        state_ = WAITING;
    }
    
    void run() {
        ros::Rate r(1.0/pub_period_);
        while (ros::ok()) {
            if (state_ != WAITING) {
                if (state_ == MOVING) 
                    updateCurrentPose(ros::Time::now());
                pose_current_.header.stamp = ros::Time::now();
                pub_pose_current_.publish(pose_current_);
                //ROS_INFO("robot_model: published currrent_pose");
            }
            ros::spinOnce();
            r.sleep();
        }
    }
    
};
}

using namespace lthmi_nav;

int main(int argc, char **argv) {
    ros::init(argc, argv, "robot_model");
    NoKinRobotModel rm;
    rm.run();
    return 0;
}
