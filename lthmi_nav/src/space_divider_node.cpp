//#define DEBUG_DIVIDER 1

#include <math.h>  
#include <string>

#include "ros/ros.h"
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseStamped.h>

#include <lthmi_nav/Experiment.h>
#include <lthmi_nav/IntMap.h>
#include <lthmi_nav/FloatMap.h>
#include <lthmi_nav/map.h>
#include <lthmi_nav/map_ros.cpp>
#include "map_divider.cpp"


using namespace lthmi_nav;



ros::NodeHandle* node;

MapRos<lthmi_nav::IntMap, int>*    my_map = NULL;
MapRos<lthmi_nav::FloatMap,float>* my_pdf = NULL;
geometry_msgs::PoseStamped*           optimal_pose = NULL;
ros::Publisher* pub1;
ros::Subscriber* sub_pdf;
ros::Subscriber* sub_opt_pose;
MapDivider* divider;
MapDivider::DivisionPolicy division_policy;
MapRos<lthmi_nav::IntMap, int>* divided_map;
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


class MapDividerDebuggerROS : public MapDividerDebugger {
public:
    float resolution_;
    MapDividerDebuggerROS(float resolution) {     
        resolution_ = resolution;
    }
    
    void text() { 
        ROS_INFO("space_divider_DEBUG: %s", msg_stream_.str().c_str()); 
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
        MapRos<lthmi_nav::IntMap, int> track_map_ros(tm);    
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





void act() {
    //ROS_INFO("space_divider: recieved opti my_map->resolution()=%f",my_map->resolution());
    Point2D center = pose2xy(optimal_pose, my_map->resolution());
    
    ROS_INFO("space_divider: will be dividing the map now, center: pose(%f, %f)m, xy(%d,%d)px", optimal_pose->pose.position.x, optimal_pose->pose.position.y, center.x, center.y);
    vector<float> optimal_priors = {0.25, 0.25, 0.25, 0.25};
    
    lthmi_nav::IntMap track_map_msg;
        track_map_msg.info.width = my_map->width();
        track_map_msg.info.height = my_map->height();
        track_map_msg.info.resolution = my_map->resolution();
        track_map_msg.data = vector<int>(my_map->width()* my_map->height(), -1);
        MapRos<lthmi_nav::IntMap, int> track_map(track_map_msg);     
    //MapRos<lthmi_nav::IntMap, int> track_map(*divided_map);
        
    divider->divide(division_policy, optimal_priors, center, my_pdf, track_map);
    pub1->publish(*(divided_map->get_msg()));
    divided_map->clean_dist();
    ROS_INFO("space_divider: published divided_map");
}

void maybeAct() {
    if (optimal_pose!=NULL && my_pdf!=NULL) {
        act();
        //my_pdf = NULL;
        optimal_pose = NULL;
    }
}

void pdfCallback(lthmi_nav::FloatMap msg) {
    my_pdf = new MapRos<lthmi_nav::FloatMap, float>(msg);
    ROS_INFO("space_divider: received pdf");
    maybeAct();
}

void poseCallback(const geometry_msgs::PoseStamped& msg) {
    optimal_pose = new geometry_msgs::PoseStamped(msg);
    ROS_INFO("space_divider: received optimal_pose");
    maybeAct();
}

void sceneCallback(lthmi_nav::Experiment msg) {
    msg.mapa.info.origin.position.x = 0;
    msg.mapa.info.origin.position.y = 0;
    my_map = new MapRos<lthmi_nav::IntMap, int>(msg.mapa);
    
    lthmi_nav::IntMap divided_map_msg;
        divided_map_msg.info.width = my_map->width();
        divided_map_msg.info.height = my_map->height();
        divided_map_msg.info.resolution = my_map->resolution();
        divided_map_msg.data = vector<int>(my_map->get_msg()->data);
        
        divided_map= new MapRos<lthmi_nav::IntMap, int>(divided_map_msg);
    divider      = new MapDivider(*my_map, *divided_map);
        #ifdef DEBUG_DIVIDER
            divider->debugger_ = new MapDividerDebuggerROS(my_map->resolution());
        #endif
    sub_pdf      = new ros::Subscriber(node->subscribe("/pdf", 10, pdfCallback));
    sub_opt_pose = new ros::Subscriber(node->subscribe("/optimal_pose", 100, poseCallback));
    ROS_INFO("space_divider: got scene");
}

bool stop(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    sub_opt_pose->shutdown();
    sub_pdf->shutdown();
    ROS_INFO("space_divider: stopped");
    return true;
};




int main(int argc, char **argv) {
    ros::init(argc, argv, "space_divider");
    
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
    ros::Publisher pub1_ =node->advertise<lthmi_nav::IntMap>("/divided_map", 1, false); //not latched
    pub1  = &pub1_;
    #ifdef DEBUG_DIVIDER
        ros::Publisher debug_pose_pub0 =node->advertise<geometry_msgs::PoseStamped>("/debug_pose", 1, true); //latched
        debug_pose_pub = &debug_pose_pub0;
        ros::Publisher debug_border_pub0 =node->advertise<geometry_msgs::PoseStamped>("/debug_border_pose", 1, true); //latched
        debug_border_pub = &debug_border_pub0;
        ros::Publisher debug_dist_map_pub0 =node->advertise<lthmi_nav::IntMap>("/debug_dist_map", 1, true); //latched
        debug_dist_map_pub = &debug_dist_map_pub0;
        ros::Publisher debug_track_map_pub0 =node->advertise<lthmi_nav::IntMap>("/debug_track_map", 1, true); //latched
        debug_track_map_pub = &debug_track_map_pub0;
        ros::Publisher debug_star_poses0 =node->advertise<geometry_msgs::PoseArray>("/debug_stars", 1, true); //latched
        debug_stars_pub = &debug_star_poses0;
    #endif
    ros::ServiceServer service = node->advertiseService("stop", stop);

    ros::spin();
    
    return 0;
}
