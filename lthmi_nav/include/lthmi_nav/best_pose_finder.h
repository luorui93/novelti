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
    Point findClosestInReachAreaEuq(Point& pt);
    Point findClosestInReachAreaObst(Point& pt);
    Point findClosestOnMap(lthmi_nav::FloatMapConstPtr pdf, Point& pt);
    virtual Point findBestPose(lthmi_nav::FloatMapConstPtr pdf) {return Point(0,0);};
    
};
}
#endif

