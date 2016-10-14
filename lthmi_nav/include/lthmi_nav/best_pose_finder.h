#ifndef LTHMI_NAV_MAP_DIVIDER_H
#define LTHMI_NAV_MAP_DIVIDER_H


//#define DEBUG_DIVIDER 1

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <lthmi_nav/IntMap.h>
#include <lthmi_nav/FloatMap.h>
#include <lthmi_nav/StartExperiment.h>
#include <lthmi_nav/common.cpp>


namespace lthmi_nav {

class BestPoseFinder : public SynchronizableNode {
public:
    double max_dist_float;
    int max_dist;
    double resolution;
    CompoundMap cmap;
    
    State state;
    ros::Publisher pub_map_div;
    ros::Subscriber sub_pose_opt;
    ros::Subscriber sub_pdf;
    
    Point cur_vertex;
    Vertex vx;
    //lthmi_nav::FloatMapConstPtr pdf;
    //boost::shared_ptr<lthmi_nav::FloatMap const> pdf;
    lthmi_nav::FloatMapConstPtr pdf;
    lthmi_nav::IntMap map_divided;
    
    std::vector<double> probs_optimal;
    std::vector<double> probs_actual;
    
    
    BestPoseFinder();
    void stop();
    void start(lthmi_nav::StartExperiment::Request& req);
    void poseOptCallback(const geometry_msgs::PoseStamped& pose);
    //void pdfCallback(lthmi_nav::FloatMapConstPtr& msg);
    //void pdfCallback(boost::shared_ptr<lthmi_nav::FloatMap const> msg){//lthmi_nav::FloatMapConstPtr& msg) {
    void pdfCallback(lthmi_nav::FloatMapConstPtr msg);/*{//lthmi_nav::FloatMapConstPtr& msg) {
        pdf=msg;
    }*/
    void divideAndPublish();
    virtual void divide(){};
    
};
}

/*
#include <lthmi_nav/map.h>
#include <lthmi_nav/map_ros.cpp>
#include "lthmi_nav/FloatMap.h"
#include <stdlib.h>

#define MEAN_DIST 1
#include "lthmi_nav/fast_dist.h"

namespace lthmi_nav {
    
    const float REACHABLE_CELL =  0.0;
    const float UNREACHABLE_CELL = -1.0;
    
    struct ReachAreaInfo {
        Point2D reach_to_main;
        Point2D reach_size;
        Point2D reach_visual;
    };
    
    //typedef vector<vector<float>> vector<vector<float>>;
    
    /*enum AnotherPixelColor {
        PIXEL_COLOR_UNREACHABLE    = 40,
        PIXEL_COLOR_REACHABLE_NOT_VISITED    = 41,
        PIXEL_COLOR_REACHABLE_VISITED    = 42
    };* /
    
    Point2D find_best_pose(
        MapIf<int>& map, 
        MapIf<float>& pdf, 
        Point2D& cur_pose, 
        int max_dist);
    
    MapRos<lthmi_nav::FloatMap,float> calculate_reachability_area(
        MapIf<int>& map, 
        Point2D& cur_pose, 
        int max_dist, 
        MapIf<float>& pdf, 
        ReachAreaInfo& rai);
    
    vector<vector<float>> generate_pdf(int width, int height);
    
    float calculate_mean_distance(MapIf<int>& map, MapIf<float>& pdf, Point2D pose_candidate, ReachAreaInfo& rai);
    //void display_float_map(vector<vector<float>>& m, int row, int col);
}
*/
