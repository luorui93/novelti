//#define DEBUG_DIVIDER 1

#include <math.h>  
#include <string>

#include "ros/ros.h"
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseStamped.h>

#include <low_throughput_hmi/Experiment.h>
#include <low_throughput_hmi/IntMap.h>
#include <low_throughput_hmi/FloatMap.h>
#include <low_throughput_hmi/map.h>
#include <low_throughput_hmi/map_ros.cpp>
#include "map_divider.cpp"


using namespace low_throughput_hmi_cost;



ros::NodeHandle* node;

MapRos<low_throughput_hmi::IntMap, int>*    my_map = NULL;
MapRos<low_throughput_hmi::FloatMap,float>* my_pdf = NULL;
geometry_msgs::PoseStamped*           optimal_pose = NULL;
ros::Publisher* pub1;
ros::Subscriber* sub_pdf;
ros::Subscriber* sub_opt_pose;
MapDivider* divider;
MapDivider::DivisionPolicy division_policy;
MapRos<low_throughput_hmi::IntMap, int>* divided_map;
Point2D* path_end_point = NULL;


Point2D pose2xy(geometry_msgs::PoseStamped* pose, float resolution) {
    return {
        (int) floor( pose->pose.position.x / resolution), 
        (int) floor( pose->pose.position.y / resolution)
    };
}

geometry_msgs::PoseStamped xy2pose(Point2D& xy, float resolution) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id="/map";
    pose.pose.position.x = (xy.x+0.5)*resolution;
    pose.pose.position.y = (xy.y+0.5)*resolution;
    return pose;
}




#ifdef DEBUG_DIVIDER

#include <geometry_msgs/PoseArray.h>
#include <tf/transform_datatypes.h>

ros::Publisher* debug_pose_pub;
ros::Publisher* debug_border_pub;
ros::Publisher* debug_dist_map_pub;
ros::Publisher* debug_track_map_pub;
ros::Publisher* debug_stars_pub;


geometry_msgs::Pose pixel2pose(Point& px, float resolution) {
    geometry_msgs::Posepose;
    pose.position.x = (px.x+0.5)*resolution;
    pose.position.y = (xy.y+0.5)px*resolution;
    return pose;
}

geometry_msgs::Pose point2pose(Point& pt, float resolution) {
    geometry_msgs::Posepose;
    pose.position.x = pt.x*resolution;
    pose.position.y = pt.y+*resolution;
    return pose;
}

geometry_msgs::PoseStamped pixel2pose(Point& px, float resolution) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id="/map";
    pose.pose.position.x = (px.x+0.5)*resolution;
    pose.pose.position.y = (xy.y+0.5)px*resolution;
    return pose;
}

geometry_msgs::PoseStamped point2pose(Point& pt, float resolution) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id="/map";
    pose.pose.position.x = pt.x*resolution;
    pose.pose.position.y = pt.y+*resolution;
    return pose;
}


class MapDividerDebuggerROS : public MapDividerDebugger {
public:
    float resolution_;
    MapDividerDebuggerROS(float resolution) {     
        resolution_ = resolution;
    }
    
    void dispay_stars(CWave2& cw) { 
        geometry_msgs::PoseArray m;
        m.header.frame_id="/map";
        for (auto s = cw.stars.begin(); s != cw.stars.end(); ++s) {
            m.poses.push_back(pixel2pose(s.c));
        }
        debug_stars_pub->publish(m);
    }
    
    void dispay_stars(CWave2& cw) { 
        geometry_msgs::PoseArray m;
        m.header.frame_id="/map";
        for (auto s = cw.stars.begin(); s != cw.stars.end(); ++s) {
            m.poses.push_back(pixel2pose(s.c));
        }
        debug_stars_pub->publish(m);
    }
    
    void pose(int x, int y) {
        //ROS_INFO("space_divider_DEBUG: border (%d,%d)",x,y); 
        Point2D pp = {x,y};
        geometry_msgs::PoseStamped p = xy2pose(pp, resolution_);
        debug_pose_pub->publish(p); 
        ros::Duration(0.01).sleep();
    }
    
    void border_pose(int x, int y, char oct) {
        //ROS_INFO("space_divider_DEBUG: border (%d,%d)",x,y); 
        Point2D pp = {x,y};
        geometry_msgs::PoseStamped p = xy2pose(pp, resolution_);
        p.pose.orientation = tf::createQuaternionMsgFromYaw((double)(oct)*M_PI/4);
        debug_border_pub->publish(p); 
        ros::Duration(0.003).sleep();
    }
    
    void dist_map() {
        ROS_INFO("space_divider_DEBUG: publishing debug distance map"); 
        debug_dist_map_pub->publish(*(my_map->get_msg())); 
        ros::Duration(0.05).sleep();
    }
    
    void track_map(MapIf<int>& tm) {
        ROS_INFO("space_divider_DEBUG: publishing debug track map"); 
        MapRos<low_throughput_hmi::IntMap, int> track_map_ros(tm);    
        track_map_ros.get_msg()->header.frame_id = "/map";
        track_map_ros.get_msg()->info.resolution = my_map->get_msg()->info.resolution;
        debug_track_map_pub->publish(*(track_map_ros.get_msg())); 
        ros::Duration(0.05).sleep();
    }
    
    void stars(vector<Point2D>& stars) {
        geometry_msgs::PoseArray m;
        m.header.frame_id="/map";
        for (int k=0; k< stars.size(); k++) {
            geometry_msgs::Pose p;
            p.position.x = (stars[k].x+0.5)*resolution_;
            p.position.y = (stars[k].y+0.5)*resolution_;
            m.poses.push_back(p);            
        }
        debug_stars_pub->publish(m);
    }
    
};


#endif





int main(int argc, char **argv) {
    ros::init(argc, argv, "test_cwave2");
    
    node = new ros::NodeHandle("~");
    string p;    
    node->getParam("division_policy", p);
    if      (p=="vert_tiles")  division_policy = MapDivider::VERT_TILES;
    else if (p=="horiz_tiles") division_policy = MapDivider::HORIZ_TILES;
    else if (p=="extremals")   division_policy = MapDivider::BY_EXTREMALS;
    else if (p=="equidists")   division_policy = MapDivider::BY_EQUIDISTS;
    else if (p=="mixed1")      division_policy = MapDivider::MIXED_METHOD1;
    else if (p=="mixed2")      division_policy = MapDivider::MIXED_METHOD2;
    else { 
        ROS_ERROR("space_divider: wrong value for 'division_policy' parameter, will die now"); 
        return 1;
    }
    
    ros::Subscriber sub1 = node->subscribe("/scene", 1, sceneCallback);
    ros::Publisher pub1_ =node->advertise<low_throughput_hmi::IntMap>("/divided_map", 1, false); //not latched
    pub1  = &pub1_;
    #ifdef DEBUG_DIVIDER
        ros::Publisher debug_pose_pub0 =node->advertise<geometry_msgs::PoseStamped>("/debug_pose", 1, true); //latched
        debug_pose_pub = &debug_pose_pub0;
        ros::Publisher debug_border_pub0 =node->advertise<geometry_msgs::PoseStamped>("/debug_border_pose", 1, true); //latched
        debug_border_pub = &debug_border_pub0;
        ros::Publisher debug_dist_map_pub0 =node->advertise<low_throughput_hmi::IntMap>("/debug_dist_map", 1, true); //latched
        debug_dist_map_pub = &debug_dist_map_pub0;
        ros::Publisher debug_track_map_pub0 =node->advertise<low_throughput_hmi::IntMap>("/debug_track_map", 1, true); //latched
        debug_track_map_pub = &debug_track_map_pub0;
        ros::Publisher debug_star_poses0 =node->advertise<geometry_msgs::PoseArray>("/debug_stars", 1, true); //latched
        debug_stars_pub = &debug_star_poses0;
    #endif
    ros::ServiceServer service = node->advertiseService("stop", stop);

    ros::spin();
    
    return 0;
}
