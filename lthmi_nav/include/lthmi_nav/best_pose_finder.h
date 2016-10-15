#ifndef LTHMI_NAV_BEST_POSE_FINDER_H
#define LTHMI_NAV_BEST_POSE_FINDER_H

//#define DEBUG_POSE_FINDER 1 

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <CompoundMap.h>
#include <CWave2.h>

#include <lthmi_nav/IntMap.h>
#include <lthmi_nav/FloatMap.h>
#include <lthmi_nav/StartExperiment.h>
#include <lthmi_nav/common.cpp>


using namespace cwave;

namespace lthmi_nav {

class BestPoseFinder : public SynchronizableNode {
public:
    const double REACH_AREA_UNASSIGNED = -1.0;
    const double REACH_AREA_UNREACHABLE = -10.0;
    
    double max_dist_float;
    int max_dist;
    double resolution;
    Point r2a;
    Point ra_min;
    Point ra_max;
    
    CompoundMap cmap;
    
    ros::Publisher pub_pose_best;
    ros::Subscriber sub_pose_cur;
    ros::Subscriber sub_pdf;
    #ifdef DEBUG_POSE_FINDER
        ros::Publisher pub_reach_area;
    #endif
    
    Point cur_vertex;
    lthmi_nav::FloatMap reach_area;
     
    BestPoseFinder();
    void start(lthmi_nav::StartExperiment::Request& req);
    void stop();
    void poseCurCallback(const geometry_msgs::PoseStamped& pose);
    void pdfCallback(lthmi_nav::FloatMapConstPtr pdf);
    void calcReachArea();
    virtual Point findBestPose(lthmi_nav::FloatMapConstPtr pdf) {return Point(0,0);};
    
};
}
#endif

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
