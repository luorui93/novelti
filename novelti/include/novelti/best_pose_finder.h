#ifndef NOVELTI_BEST_POSE_FINDER_H
#define NOVELTI_BEST_POSE_FINDER_H

#include <mutex>
#include <string>

#include <ros/ros.h>

#if ROSCONSOLE_MIN_SEVERITY == ROSCONSOLE_SEVERITY_DEBUG
    #define DEBUG_POSE_FINDER 1
#endif

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <CompoundMap.h>
#include <CWave2.h>

#include <novelti/IntMap.h>
#include <novelti/FloatMap.h>
#include <novelti/StartExperiment.h>
//#include <novelti/common.cpp>
#include <novelti/inference_unit.h>

#ifdef DEBUG_POSE_FINDER
    #define PUB_DEBUG_POSE(x,y, wrtMap) pubDebugPose((x),(y), (wrtMap))
#else
    #define PUB_DEBUG_POSE(x,y, wrtMap)
#endif


using namespace cwave;

namespace novelti {

class BestPoseFinder{
public:
    ros::NodeHandle node;
    const double REACH_AREA_UNASSIGNED = -1.0;
    const double REACH_AREA_UNREACHABLE = -10.0;
    
    double max_dist_float;
    int max_dist;
    double resolution;
    int pose_to_vertex_tolerance;
    Point r2a;
    Point ra_min;
    Point ra_max;
    long int n_unassigned;
    Point pt; 
    bool isNode;
    
    CompoundMap cmap;
    
    ros::Publisher pub_pose_best;
    ros::Subscriber sub_pose_cur;
    ros::Subscriber sub_pdf;
    #ifdef DEBUG_POSE_FINDER
        ros::Publisher pub_reach_area;
        ros::Publisher pub_pose_debug;
        void pubDebugPose(int x, int y, bool wrtMap);
    #endif
    
    //Point cur_vertex;
    geometry_msgs::Pose pose_current;
    std::mutex pose_current_lock_;
    geometry_msgs::PoseStamped pose_best;
    novelti::FloatMap reach_area;
    const novelti::FloatMap* pdf_;
     
    BestPoseFinder();
    BestPoseFinder(const std::string paramPrefix);
    void startExp(novelti::StartExperiment::Request& req);
    void stopExp();
    void poseCurCallback(geometry_msgs::PoseStamped pose);
    void calculate(const novelti::FloatMap& pdf);
    void publish();
    void pdfCallback(novelti::FloatMapConstPtr pdf);
    
protected:    
    bool getCurVertex(int& cx, int& cy);
    bool calcReachArea();
    void moveToClosestInReachAreaEuc();
    void moveToClosestInReachAreaObst();
    void moveToClosestOnMap();
    virtual void findBestPose();
    bool isOnBorder(Point p);
    void updatePose(geometry_msgs::PoseStamped& pose, int x, int y);
};
}
#endif

