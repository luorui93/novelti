#ifndef SPACE_DIVIDER_H_
#define SPACE_DIVIDER_H_


#include <low_throughput_hmi/fast_dist.h>
#include <math.h>
#include <cmath>
#include <ncurses.h>

#define DEBUG_IN_ROSdd 1

#include "ros/ros.h"

namespace low_throughput_hmi_cost {

    extern ros::Publisher* pub_debug_pose;
    
    typedef unsigned long RegionSize;
    
    typedef struct {
        int x;
        int y;
        char oct;
        int dist;
        int cur_star;
    } BorderWalker;
    
    typedef struct {
        MapIf<int>& map;
        MapIf<float>& pdf;
        MapIf<int>& track_map;
        vector<Point2D> track_stars;
        BorderWalker walker;
        MapIf<int>& divided_map;
        vector<float> opt_prob_threshs;
        float cur_prob;
        char cur_region;
        #ifdef DEBUG_IN_ROS
            ros::Publisher* pub_debug_pose;
        #endif
    } SpaceDivider;
    

    Point2D* map_divide(
            vector<float> opt_probs, 
            MapIf<int>& map, 
            MapIf<float>& pdf, 
            Point2D& c,
            MapIf<int>& track_map, 
            MapIf<int>& divided_map
    );
    
    Point2D* divide(
            vector<float> opt_probs, 
            MapIf<int>& map, 
            MapIf<float>& pdf, 
            MapIf<int>& track_map, 
            MapIf<int>& divided_map, 
            vector<Point2D>& track_stars, 
            Point2D center
    );

    Point2D find_border_point(SpaceDivider &sd, Point2D& p);

    void border_step(SpaceDivider &sd, BorderWalker& w);
    
    void path_walk(SpaceDivider &sd, int x, int y);
    
    void line_walk(SpaceDivider &sd, Point2D s, Point2D e);
    
    void pixel_process(SpaceDivider &sd, int x, int y);
    
}
#endif