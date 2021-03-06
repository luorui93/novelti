#ifndef NOVELTI_MAP_DIVIDER_H
#define NOVELTI_MAP_DIVIDER_H

#include <ros/ros.h>

#if ROSCONSOLE_MIN_SEVERITY == ROSCONSOLE_SEVERITY_DEBUG
    #define DEBUG_DIVIDER 1
#endif



#include <geometry_msgs/PoseStamped.h>
#include <string>

#include <novelti/IntMap.h>
#include <novelti/FloatMap.h>
#include <novelti/StartExperiment.h>
//#include <novelti/common.cpp>
#include <CWave2.h>
#include <novelti/Command.h>
#include <novelti/inference_matrix.h>

namespace novelti {

using namespace cwave;
    
class MapDivider{
public:
    enum State { WAITING, ONLY_POSE, ONLY_PDF };
    State state;
    ros::NodeHandle node;
    ros::Publisher pub_map_div;
    ros::Publisher pub_selection_highlight;
    ros::Subscriber sub_pose_opt;
    ros::Subscriber sub_pdf;
    
    const novelti::FloatMap*           pdf;
    const geometry_msgs::PoseStamped* pose_best;
    Point                              pt_best;
    novelti::IntMap                  map_divided;
    novelti::IntMap                  transparent_map, selection_highlight;
    
    std::vector<double> probs_optimal;
    std::vector<double> probs_actual;
    
    #ifdef DEBUG_DIVIDER
        ros::Publisher pub_debug_pose;
        ros::Publisher pub_debug_map_div;
        void publishDebugPose(int x, int y);
    #endif
    bool isNode;
    
    MapDivider();
    MapDivider(const std::string paramPrefix);
    virtual void stopExp();
    virtual void startExp(novelti::StartExperiment::Request& req);
    void poseOptCallback(geometry_msgs::PoseStampedConstPtr msg);
    //void pdfCallback(novelti::FloatMapConstPtr& msg);
    //void pdfCallback(boost::shared_ptr<novelti::FloatMap const> msg){//novelti::FloatMapConstPtr& msg) {
    void pdfCallback(novelti::FloatMapConstPtr msg);
    //void noveltiMapCallback(novelti::FloatMapConstPtr ptr_pdf, geometry_msgs::PoseStampedConstPtr ptr_pose);
    void dividePublishMap(const novelti::FloatMap& pdf0, const geometry_msgs::PoseStamped& pose);
    void divide(const novelti::FloatMap& pdf0, const geometry_msgs::PoseStamped& pose);
    void publish();
    void highlightSelection(int cmd);
    void divideAndPublish();
    void clearAndPublish();
    std::vector<double> probs_scaled;
    int cur_region;
    double prob;
    void startDivider();
    void updateProbsScaled();
    void markVertex(int x, int y);
    void markVertex(int k);
    void endDivider();
    
    virtual void divide(){};
    
};
}

#endif