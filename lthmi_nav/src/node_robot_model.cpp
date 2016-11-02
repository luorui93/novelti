/*
     subs                                       pubs
                    +--------------------+
 /pose_desired ---> |                    | ---> /pose_current
                    |  node_robot_model  | 
/pose_inferred ---> |                    | ---> /pose_arrived
                    +--------------------+
                               ^
                               |
                           srv: start
                               req:  map
                                   init_pose
                               resp: -
*/


#include <atomic>

#include <CompoundMap.h>
#include <CWave2.h>

#include "ros/ros.h"
#include <math.h>
#include <geometry_msgs/PoseStamped.h>

#include <lthmi_nav/StartExperiment.h>
#include <lthmi_nav/common.cpp>

#include <mutex>

using namespace cwave;

namespace lthmi_nav {

class NoKinRobotModel : public SynchronizableNode {
public:
    ros::Publisher  pub_pose_current_;
    ros::Publisher  pub_pose_arrived_;
    ros::Subscriber sub_pose_desired_;
    ros::Subscriber sub_pose_inferred_;
    std::mutex pub_lock_;
    std::atomic<bool> active_;
    
    geometry_msgs::PoseStamped pose_;
    std::mutex pose_lock_;

    CompoundMap cmap_;
    struct Traj {
        vector<Point> path; //[destination, ... ... ,source]
        ros::Time start_time;
        bool moving;
    };
    Traj traj_;
    std::mutex traj_lock_;
    
    double max_vel_, max_vel_ang_, pub_period_;

    NoKinRobotModel() :
        SynchronizableNode()
    {
        node.param("max_vel", max_vel_,  0.5);
        node.param("max_vel_ang", max_vel_ang_, 0.5); //currently ignored
        node.param("pub_period", pub_period_, 0.025);
        pose_.header.frame_id = "/map";
        active_ = false;
    }
    
    void start(lthmi_nav::StartExperiment::Request& req) {
        pose_lock_.lock();
            pose_.pose  = req.init_pose;
        pose_lock_.unlock();
        
        traj_lock_.lock();
        new (&cmap_) CompoundMap(req.map.info.width, req.map.info.height);
        for (int x=0; x<req.map.info.width; x++)
            for (int y=0; y<req.map.info.height; y++)
                if (req.map.data[x + y*req.map.info.width]==0)
                    cmap_.setPixel(x,y, FREED); //free
        traj_lock_.unlock();
                    
        pub_lock_.lock();
            pub_pose_current_  = node.advertise<geometry_msgs::PoseStamped>("/pose_current", 1, false); //not latched
            pub_pose_arrived_  = node.advertise<geometry_msgs::PoseStamped>("/pose_arrived", 1, false); //not latched
            sub_pose_desired_  = node.subscribe("/pose_desired", 1, &NoKinRobotModel::desiredPoseCallback, this);
            sub_pose_inferred_ = node.subscribe("/pose_inferred", 1, &NoKinRobotModel::desiredPoseCallback, this);
        pub_lock_.unlock();
        publishCurrentPose();
        active_ = true;
    }
    
    void stop() {
        pub_lock_.lock();
            active_ = false;
        pub_lock_.unlock();
        sub_pose_desired_.shutdown();
        sub_pose_inferred_.shutdown();
        pub_pose_current_.shutdown();
        pub_pose_arrived_.shutdown();
    }
    
    void desiredPoseCallback(geometry_msgs::PoseStampedConstPtr pose_des) {
        ROS_INFO("robot_model: recieved desired pose: (%f,%f)", pose_des->pose.position.x, pose_des->pose.position.y);
        Point src;
        pose_lock_.lock();
            updateVertex(pose_.pose, src.x, src.y);
        pose_lock_.unlock();
        traj_lock_.lock();
            traj_.start_time = ros::Time::now();
            
            CWave2 cw(cmap_);
            CWave2Processor dummy;
            cw.setProcessor(&dummy);
            cw.calc(src);
            
            Point p;
            updateVertex(pose_des->pose, p.x, p.y);
            traj_.path.clear();
            traj_.path.push_back(p);
            TrackStar tstar;
            int track_star_id;
            do {
                track_star_id = cmap_.getTrackStarId(p.x,p.y);
                tstar = cmap_.getTrackStar(track_star_id);
                p = Point(tstar.x,tstar.y);
                traj_.path.push_back(p);      //ROS_INFO("robot_model: added: %d,%d", p.x, p.y);
            } while(track_star_id!=0);
            cmap_.clearDist();
            traj_.moving = true;
        traj_lock_.unlock();
    }

    void publishCurrentPose() {
        pose_lock_.lock();
            pose_.header.stamp = ros::Time::now();
            pub_pose_current_.publish(pose_);
        pose_lock_.unlock();
    }
    
    bool updateCurrentPose(ros::Time t) {
        // input: traj, t,
        // output: x,y
        // return true if arrived
        double T, dt_seg, gamma;
        Point cur,nxt;
        bool need_update, arrived=false;
        traj_lock_.lock();
            need_update = traj_.moving;
            if (need_update) {
                T = (t-traj_.start_time).toSec();
                traj_.moving = false;
                for (int k=traj_.path.size()-1; k>=1; k-- ) {
                    cur = traj_.path[k];
                    nxt = traj_.path[k-1];
                    dt_seg = sqrt( (nxt.x-cur.x)*(nxt.x-cur.x) + (nxt.y-cur.y)*(nxt.y-cur.y) )*resolution/max_vel_;
                    if (T <= dt_seg) {
                        gamma = T/dt_seg;
                        traj_.moving = true;
                        break;
                    }
                    T -= dt_seg;
                }
                if (!traj_.moving) {
                    gamma = 1.0;
                    arrived  = true;
                }
            }
        traj_lock_.unlock();
        if (need_update) {
            pose_lock_.lock();
                pose_.pose.position.x = resolution * (cur.x + (nxt.x-cur.x)*gamma);
                pose_.pose.position.y = resolution * (cur.y + (nxt.y-cur.y)*gamma);
            pose_lock_.unlock();
        }
        return arrived;
    }
    
    
    void publishPoseArrived() {
        pose_lock_.lock();
            pose_.header.stamp = ros::Time::now();
            pub_pose_arrived_.publish(pose_);
        pose_lock_.unlock();
        ROS_INFO("%s: published /pose_arrived (%f,%f)", getName().c_str(), pose_.pose.position.x, pose_.pose.position.y);
    }
    
    void run() {
        ros::Rate r(1.0/pub_period_);
        while (ros::ok()) {
            if (active_) {
                pub_lock_.lock();
                    if (updateCurrentPose(ros::Time::now()))
                        publishPoseArrived();
                    publishCurrentPose();
                    //ROS_INFO("robot_model: published currrent_pose: (%f,%f)", pose_current_.pose.position.x, pose_current_.pose.position.y);
                pub_lock_.unlock();
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
