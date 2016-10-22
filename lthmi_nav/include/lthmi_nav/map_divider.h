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

class MapDivider : public SynchronizableNode {
public:
    enum State { WAITING, ONLY_POSE, ONLY_PDF };
    State state;
    ros::Publisher pub_map_div;
    ros::Subscriber sub_pose_opt;
    ros::Subscriber sub_pdf;
    
    Vertex                             vx;
    lthmi_nav::FloatMapConstPtr        pdf;
    geometry_msgs::PoseStampedConstPtr pose_best;
    lthmi_nav::IntMap                  map_divided;
    
    std::vector<double> probs_optimal;
    std::vector<double> probs_actual;
    
    
    MapDivider();
    void stop();
    void start(lthmi_nav::StartExperiment::Request& req);
    void poseOptCallback(geometry_msgs::PoseStampedConstPtr pose);
    //void pdfCallback(lthmi_nav::FloatMapConstPtr& msg);
    //void pdfCallback(boost::shared_ptr<lthmi_nav::FloatMap const> msg){//lthmi_nav::FloatMapConstPtr& msg) {
    void pdfCallback(lthmi_nav::FloatMapConstPtr msg);/*{//lthmi_nav::FloatMapConstPtr& msg) {
        pdf=msg;
    }*/
    void divideAndPublish();
    virtual void divide(){};
    
};
}

#endif