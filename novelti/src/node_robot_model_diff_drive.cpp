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
#include <geometry_msgs/Quaternion.h>
#include <novelti/StartExperiment.h>
#include <novelti/common.cpp>

#include <mutex>
#include <queue>

using namespace cwave;

namespace novelti {

class DiffDriveRobotModel : public SynchronizableNode {
public:
    ros::Publisher  pub_pose_current_;
    ros::Publisher  pub_pose_arrived_;
    ros::Subscriber sub_pose_desired_;
    ros::Subscriber sub_position_desired_;
    ros::Subscriber sub_orientation_desired_;
    std::mutex pub_lock_;
    std::atomic<bool> active_;
    
    geometry_msgs::PoseStamped pose_current_,pose_last_;
    std::mutex pose_lock_;

    CompoundMap cmap_;

    std::mutex trans_lock_;
    bool traj_moving;

    struct TransitionPoint {
        enum Type {initial, rotation, translation};
        Type prim_id;
        double start_time;      //time when desired_pose is received
        double finish_time;     //time when current movement is finished
        double period;          //time to finish current movement;
        Point cur_pos;
        Point prev_pos;
        double cur_orientation;     //only in Yaw 
        double prev_orientation;
    };
    queue<TransitionPoint> trans_point_queue_;

    double max_vel_, max_twist_, pub_period_, initial_orientation_;

    DiffDriveRobotModel() :
        SynchronizableNode()
    {
        node.param("max_vel", max_vel_,  0.5);
        node.param("max_twist", max_twist_, 0.5);
        node.param("pub_period", pub_period_, 0.01);
        node.param("initial_orientation", initial_orientation_, 0.0);
        pose_current_.header.frame_id = "/map";
        active_ = false;
    }
    
    void start(novelti::StartExperiment::Request& req) {
        pose_lock_.lock();
            pose_current_.pose  = req.init_pose;
            pose_current_.pose.orientation = tf::createQuaternionMsgFromYaw(initial_orientation_);
        pose_lock_.unlock();
        
        trans_lock_.lock();
            traj_moving = false;
            new (&cmap_) CompoundMap(req.map.info.width, req.map.info.height);
            for (int x=0; x<req.map.info.width; x++)
                for (int y=0; y<req.map.info.height; y++)
                    if (req.map.data[x + y*req.map.info.width]==0)
                        cmap_.setPixel(x,y, FREED); //free
        trans_lock_.unlock();
                    
        pub_lock_.lock();
            pub_pose_current_  = node.advertise<geometry_msgs::PoseStamped>("/pose_current", 1, false); //not latched
            pub_pose_arrived_  = node.advertise<geometry_msgs::PoseStamped>("/pose_arrived", 1, true); //latched
            sub_position_desired_ = node.subscribe("/position_desired", 1, &DiffDriveRobotModel::desiredPositionCallback, this);
            sub_pose_desired_ = node.subscribe("/pose_desired", 1, &DiffDriveRobotModel::desiredPoseCallback, this);
        pub_lock_.unlock();
        publishCurrentPose();
        active_ = true;
    }
    
    void stop() {
        pub_lock_.lock();
            active_ = false;
        pub_lock_.unlock();
        sub_pose_desired_.shutdown();
        sub_position_desired_.shutdown();
        pub_pose_current_.shutdown();
        pub_pose_arrived_.shutdown();
    }
    
    void desiredPositionCallback(geometry_msgs::PoseStampedConstPtr pose_des) {
        ROS_WARN("robot_model: recieved desired position: (%f,%f)", pose_des->pose.position.x, pose_des->pose.position.y);
        Point des;
        pose_lock_.lock();
            updateVertex(pose_des->pose, des.x, des.y);
        pose_lock_.unlock();
        trans_lock_.lock();
            //clear the previous queue
            std::queue<TransitionPoint> empty;
            std::swap(trans_point_queue_, empty);
            // ROS_WARN("queue cleared, the new size is %d", int(trans_point_queue_.size()));
            TransitionPoint trans_point;
            trans_point.start_time = ros::Time::now().toSec();
            trans_point.finish_time = trans_point.start_time;
            trans_point.prim_id = TransitionPoint::Type::initial;
            Point p;
            updateVertex(pose_current_.pose, p.x, p.y);
            trans_point.cur_pos = p;
            trans_point.cur_orientation = tf::getYaw(pose_current_.pose.orientation);
            trans_point_queue_.push(trans_point);

            CWave2 cw(cmap_);
            CWave2Processor dummy;
            cw.setProcessor(&dummy);
            cw.calc(des);
            
            TrackStar tstar;    
            int track_star_id;
            double theta,period;
            do {
                track_star_id = cmap_.getTrackStarId(p.x,p.y);
                tstar = cmap_.getTrackStar(track_star_id);
                
                //Rotation
                if (tstar.y == p.y && tstar.x == p.x)
                    break;
                theta = atan2(tstar.y-p.y,tstar.x-p.x) - trans_point.cur_orientation;
                //restrict theta to -pi to pi
                if (theta > M_PI) {
                    theta = 2*M_PI - theta;
                }
                else if (theta < -M_PI) {
                    theta = 2*M_PI + theta;
                }
                period = abs(theta) / max_twist_;
                trans_point.prim_id = TransitionPoint::Type::rotation;
                trans_point.finish_time += period;
                trans_point.period = period;
                // ROS_WARN("queue added period:%f",period);
                trans_point.prev_orientation = trans_point.cur_orientation;
                trans_point.cur_orientation = atan2(tstar.y-p.y,tstar.x-p.x);
                trans_point_queue_.push(trans_point);

                //Translation
                period = sqrt( (tstar.x-p.x)*(tstar.x-p.x) + (tstar.y-p.y)*(tstar.y-p.y) )*resolution/max_vel_;
                trans_point.prim_id = TransitionPoint::Type::translation;
                trans_point.finish_time += period;
                trans_point.period = period;
                trans_point.prev_pos = trans_point.cur_pos;
                trans_point.cur_pos = Point(tstar.x, tstar.y);
                trans_point_queue_.push(trans_point);      //ROS_INFO("robot_model: added: %d,%d", p.x, p.y);

                p = Point(tstar.x,tstar.y);
            } while(track_star_id!=0);    
            //pop out the initial value
            trans_point_queue_.pop();    
            //ROS_WARN("Last element in the queue: %d", trans_point_queue_.back().prim_id);
            cmap_.clearDist();
            traj_moving = true;
        trans_lock_.unlock();
    }

    void desiredPoseCallback(geometry_msgs::PoseStampedConstPtr pose_des) {
        desiredPositionCallback(pose_des);
        ROS_WARN("robot_model: recieved desired orientation: %f",tf::getYaw(pose_des->pose.orientation));
        //Do one more final rotation
        TransitionPoint trans_point;
        double theta,period;
        trans_lock_.lock();
        theta = tf::getYaw(pose_des->pose.orientation) - trans_point_queue_.back().cur_orientation;
        //ROS_WARN("theta: %f",theta);
        //restrict theta to -pi to pi
        if (theta > M_PI) {
            theta = 2*M_PI - theta;
        }
        else if (theta < -M_PI) {
            theta = 2*M_PI + theta;
        }
        period = abs(theta) / max_twist_;            
        trans_point.prim_id = TransitionPoint::Type::rotation;
        trans_point.start_time = trans_point_queue_.back().finish_time;
        trans_point.finish_time =  trans_point.start_time+period;
        trans_point.period = period;
        trans_point.prev_orientation = trans_point_queue_.back().cur_orientation;
        trans_point.cur_orientation = tf::getYaw(pose_des->pose.orientation);
        trans_point.cur_pos = trans_point_queue_.back().cur_pos;
        trans_point.prev_pos = trans_point_queue_.back().prev_pos;
        trans_point_queue_.push(trans_point);      
        ROS_WARN("Last element in the queue: %f", trans_point_queue_.back().cur_orientation);
        traj_moving = true;
        trans_lock_.unlock(); 
    }

    void publishCurrentPose() {
        pose_lock_.lock();
            pose_current_.header.stamp = ros::Time::now();
            pub_pose_current_.publish(pose_current_);
            //ROS_INFO("robot_model: published currrent_pose: (%f,%f)", pose_.pose.position.x, pose_.pose.position.y);
        pose_lock_.unlock();
    }
    
    bool updateCurrentPose(ros::Time t) {
        // input: traj, t,
        // output: x,y
        // return true if arrived
        double T, theta, gamma;
        bool need_update, arrived=false;
        TransitionPoint::Type type;
        trans_lock_.lock();
            TransitionPoint tp;
            need_update = traj_moving;
            if (need_update) {
                if (!trans_point_queue_.empty()) {
                    tp = trans_point_queue_.front();
                    //ROS_WARN("current tp orientation:%f",tp.cur_orientation);
                    T = tp.finish_time - t.toSec();
                    if (T >= 0) {
                        gamma = 1 - T / tp.period;
                        // ROS_WARN("current element: %d", tp.prim_id);
                        // ROS_WARN("period: %f",tp.period);
                        traj_moving = true;
                        type = tp.prim_id;
                        if (type == TransitionPoint::Type::rotation) {
                            pose_lock_.lock();
                            theta = tp.cur_orientation - tp.prev_orientation;
                            //Restrict theta from -pi to pi
                            if (theta > M_PI) {
                                theta = -(2 * M_PI - theta);
                            }
                            else if (theta < -M_PI) {
                                theta = 2 * M_PI + theta;
                            }
                            pose_current_.pose.orientation = tf::createQuaternionMsgFromYaw(
                                tp.prev_orientation + theta * gamma);
                            pose_lock_.unlock();
                        }
                        else if (type == TransitionPoint::Type::translation) {
                            pose_lock_.lock();
                            pose_current_.pose.position.x = resolution * (tp.prev_pos.x + (tp.cur_pos.x - tp.prev_pos.x) * gamma);
                            pose_current_.pose.position.y = resolution * (tp.prev_pos.y + (tp.cur_pos.y - tp.prev_pos.y) * gamma);
                            pose_lock_.unlock();
                        }
                    }

                    else {
                        pose_lock_.lock();
                        pose_current_.pose.position.x = resolution * (tp.cur_pos.x);
                        pose_current_.pose.position.y = resolution * (tp.cur_pos.y);
                        pose_lock_.unlock();                        
                        trans_point_queue_.pop(); //pop out the top element if the stage has completed
                    }
                }
                else {
                    traj_moving = false;
                    arrived = true;
             }
            }
        trans_lock_.unlock();
        return arrived;
    }
    
    
    void publishPoseArrived() {
        pose_lock_.lock();
            pose_current_.header.stamp = ros::Time::now();
            pose_last_ = pose_current_;
            pub_pose_arrived_.publish(pose_current_);
        pose_lock_.unlock();
        ROS_INFO("%s: published /pose_arrived (%f,%f)", getName().c_str(), pose_current_.pose.position.x, pose_current_.pose.position.y);
    }

    void run() {
        ros::Rate r(1.0/pub_period_);
        while (ros::ok()) {
            if (active_) {
                pub_lock_.lock();
                    if (updateCurrentPose(ros::Time::now()))
                        publishPoseArrived();
                    publishCurrentPose();
                pub_lock_.unlock();
            }
            ros::spinOnce();
            r.sleep();
        }
    }
};
} //namespace novelti



using namespace novelti;

int main(int argc, char **argv) {
    ros::init(argc, argv, "robot_model");
    DiffDriveRobotModel rm;
    rm.run();
    return 0;
}
