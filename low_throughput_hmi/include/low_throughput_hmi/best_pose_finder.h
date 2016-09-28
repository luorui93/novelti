#include <low_throughput_hmi/map.h>
#include <low_throughput_hmi/map_ros.cpp>
#include "low_throughput_hmi/FloatMap.h"
#include <stdlib.h>

#define MEAN_DIST 1
#include "low_throughput_hmi/fast_dist.h"

namespace low_throughput_hmi_cost {
    
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
    };*/
    
    Point2D find_best_pose(
        MapIf<int>& map, 
        MapIf<float>& pdf, 
        Point2D& cur_pose, 
        int max_dist);
    
    MapRos<low_throughput_hmi::FloatMap,float> calculate_reachability_area(
        MapIf<int>& map, 
        Point2D& cur_pose, 
        int max_dist, 
        MapIf<float>& pdf, 
        ReachAreaInfo& rai);
    
    vector<vector<float>> generate_pdf(int width, int height);
    
    float calculate_mean_distance(MapIf<int>& map, MapIf<float>& pdf, Point2D pose_candidate, ReachAreaInfo& rai);
    //void display_float_map(vector<vector<float>>& m, int row, int col);
}

