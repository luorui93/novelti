#define DEBUG_POSE_FINDER 1

#include "ros/ros.h"
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <lthmi_nav/Experiment.h>
#include <lthmi_nav/IntMap.h>
#include <lthmi_nav/FloatMap.h>
#include <lthmi_nav/map_ros.cpp>
#include <math.h>  
#include "opt_pose_finder.cpp"
//#include <time.h>

using namespace lthmi_nav;

ros::NodeHandle* node;

//publishers and subscribers
ros::Publisher*  pub_opt;
ros::Publisher*  pub_div;
ros::Subscriber* sub_pdf;
ros::Subscriber* sub_cur_pose;

//ROS parameters
float max_dist;

//others
MapRos<lthmi_nav::IntMap, int>*    my_map     = NULL;
MapRos<lthmi_nav::FloatMap,float>* my_pdf     = NULL;
geometry_msgs::PoseStamped*           current_pose = NULL;

OptimalPoseFinder* pose_finder;
MapRos<lthmi_nav::FloatMap,float>* reach_area;

OptimalPoseFinder::Method pose_method;


#ifdef DEBUG_POSE_FINDER
ros::Publisher* reach_area_pub;

class OptimalPoseFinderDebuggerROS : public OptimalPoseFinderDebugger {
public:
    float resolution_;
    ros::Publisher* pose_publisher_;
    OptimalPoseFinderDebuggerROS(/*ros::Publisher* pose_publisher, float resolution*/) {
        //pose_publisher_ = pose_publisher;
        //resolution_ = resolution;
    }
    
    void mean_dist_map() { 
        reach_area->get_msg()->header.frame_id="/map";
        reach_area->get_msg()->info.width = my_map->width();
        reach_area->get_msg()->info.height = my_map->height();
        reach_area->get_msg()->info.resolution = my_map->resolution();        
        reach_area_pub->publish(*(reach_area->get_msg()));
    }
    
    void text() { 
        ROS_INFO("best_pose_finder_DEBUG: %s", msg_stream_.str().c_str()); 
    }
    /*
    void pose(int x, int y) {
        //ROS_INFO("space_divider_DEBUG: border (%d,%d)",x,y); 
        geometry_msgs::PoseStamped p;
        p.header.frame_id="/map";
        p.pose.position.x = (x+0.5)*resolution_;
        p.pose.position.y = (y+0.5)*resolution_;
        pose_publisher_->publish(p);
        ros::Duration(0.01).sleep();
    }*/
};
#endif


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

void act() {
    ROS_INFO("best_pose_finder: will find the best pose now");
    Point2D xy = pose2xy(current_pose, my_map->resolution());
    //ROS_INFO("best_pose_finder: central point=(%f,%f), in cells=(%d,%d)", current_pose->pose.position.x, current_pose->pose.position.y, xy.x, xy.y);
    Point2D opt_xy = pose_finder->find(xy, *my_pdf);
    geometry_msgs::PoseStamped opt_pose = xy2pose(opt_xy, my_map->resolution());
    pub_opt->publish(opt_pose);
    ROS_INFO("best_pose_finder: published optimal pose(%f,%f) = xy(%d,%d)", opt_pose.pose.position.x, opt_pose.pose.position.y, opt_xy.x, opt_xy.y);
}

void maybeAct() {
    if (current_pose!=NULL && my_pdf!=NULL) {
        act();
        my_pdf = NULL;
        current_pose = NULL;
    }
}

void pdfCallback(lthmi_nav::FloatMap msg) {
    my_pdf = new MapRos<lthmi_nav::FloatMap, float>(msg);
    ROS_INFO("best_pose_finder: received pdf");
    maybeAct();
}

void poseCallback(const geometry_msgs::PoseStamped& msg) {
    current_pose = new geometry_msgs::PoseStamped(msg);
    //ROS_INFO("best_pose_finder: received current_pose"); 
    maybeAct();
}

void sceneCallback(lthmi_nav::Experiment msg) {
    ROS_INFO("best_pose_finder: got scene");
    my_map       = new MapRos<lthmi_nav::IntMap, int>(msg.mapa);
    sub_pdf      = new ros::Subscriber(node->subscribe("/pdf", 10, pdfCallback));
    sub_cur_pose = new ros::Subscriber(node->subscribe("/current_pose", 100, poseCallback));
    reach_area   = new MapRos<lthmi_nav::FloatMap, float>(0,0,0);
    pose_finder  = new OptimalPoseFinder(pose_method, *my_map, (int)(max_dist/my_map->resolution()), *reach_area);
        #ifdef DEBUG_POSE_FINDER
            pose_finder->debugger_ = new OptimalPoseFinderDebuggerROS();
        #endif
    //ROS_INFO("best_pose_finder: digested scene");
}


bool stop(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    sub_cur_pose->shutdown();
    sub_pdf->shutdown();
    ROS_INFO("best_pose_finder: stopped");
    return true;
};



int main(int argc, char **argv) {   
    ros::init(argc, argv, "best_pose_finder");
    
    node = new ros::NodeHandle("~");
    
    double max_velocity, period, velocity_safety_coef;
    node->getParam("max_velocity", max_velocity);
    node->getParam("velocity_safety_coef", velocity_safety_coef);
    node->getParam("period", period);
    string method;
    node->getParam("method", method);
    ROS_INFO("best_pose_finder: method=%s", method.c_str());
    if      (method=="opt")      pose_method = OptimalPoseFinder::OPT;
    else if (method=="localopt") pose_method = OptimalPoseFinder::LOCALOPT;
    else if (method=="maxprob")  pose_method = OptimalPoseFinder::MAXPROB;
    else if (method=="tomaxprob") pose_method = OptimalPoseFinder::TOMAXPROB;
    else if (method=="cog")      pose_method = OptimalPoseFinder::COG;
    else if (method=="cog2")      pose_method = OptimalPoseFinder::COG2;
    else if (method=="cog2lopt")      pose_method = OptimalPoseFinder::COG2LOPT;
    else if (method=="all3cog")      pose_method = OptimalPoseFinder::ALL3COG;
    else if (method=="all3lopt")      pose_method = OptimalPoseFinder::ALL3LOPT;
    else if (method=="all3gopt")      pose_method = OptimalPoseFinder::ALL3GOPT;
    else { 
        ROS_ERROR("best_pose_finder: wrong value for 'method' parameter, will die now"); 
        return 1;
    }
    max_dist = velocity_safety_coef*max_velocity*period;
    ros::Subscriber sub = node->subscribe("/scene", 1, sceneCallback);
    ros::Publisher pub1 = 
        node->advertise<geometry_msgs::PoseStamped>("/optimal_pose", 1, false); //not latched
    pub_opt  = &pub1;
    #ifdef DEBUG_POSE_FINDER
        ros::Publisher pub2 = node->advertise<lthmi_nav::FloatMap>("/reach_area", 1, false); //not latched
        reach_area_pub  = &pub2;
    #endif
    
    ros::ServiceServer service = node->advertiseService("stop", stop);
    ros::spin();
    return 0;
}
