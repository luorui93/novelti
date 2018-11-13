#ifndef NOVELTI_POSITION_CONTROL_H
#define NOVELTI_POSITION_CONTROL_H


#include <novelti/inference_unit.h>
#include <vector>
#include <ros/ros.h>
#include <novelti/Command.h>
#include <novelti/IntMap.h>
#include <novelti/FloatMap.h>
#include <geometry_msgs/PoseStamped.h>
#include <novelti/pdf_utils.h>
#include <novelti/best_pose_finder.h>
#include <novelti/map_divider.h>

namespace novelti {

struct POI {
   double x;
   double y;
   double sigma;
   double k; //weight in the sum of gaussians
};


class PositionControl : public InferenceUnit {
public:
    const float PDF_UNREACHABLE = -10.0;

    PositionControl(ros::NodeHandle& node);
    ~PositionControl();
    void start(StartExperiment::Request& req);
    void stop();    
    void initPriors(std::vector<double>& new_priors);
    void update(const std::vector<double>& coefs,  const int cmd, std::vector<double>& new_priors);
    void onInferred(int inferredCmd);
    void act();
    
    const geometry_msgs::PoseStamped& getPositionInferred() const;

protected:
    ros::NodeHandle& node_;
    BestPoseFinder* best_position_finder_;
    MapDivider*     map_divider_;   
    ros::Publisher  pub_pdf_;
    ros::Publisher  pub_position_inferred_;
    ros::Publisher  pub_position_desired_;
    FloatMap        pdf_;
    geometry_msgs::PoseStamped position_inferred_;

    float interest_area_thresh_;    
    std::vector<int> view_sizes_;
    int view_size_id_;
    std::vector<int> smooth_rads_;

    float uniform_prob_;
    bool  reset_pdf_on_new_;
    std::vector<POI> pois_;    
    bool  new_pdf_;
    float eps_;
    PdfStats<float> stats_;
    
    std::vector<int> m;//tmp
    
    const std::vector<int>& getIndexMap();
    void readPOIsParam();
    void setPdfFromPOIs();
    void publishPdf();
    void publishPositionInferred();
    bool needsSmoothening();
    bool isSmoothableVertex(int cx, int cy, int smooth_rad);
    void smoothen();
    void publishViewTf();
    
    BestPoseFinder* newBestPositionFinder();
    MapDivider* newMapDivider();
};
}

#endif