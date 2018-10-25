#ifndef NOVELTI_ORIENTATION_CONTROL_H
#define NOVELTI_ORIENTATION_CONTROL_H

#include <vector>
#include <string>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

#include <novelti/IntMap.h>
#include <novelti/FloatMap.h>
#include <novelti/OrientationPdf.h>
#include <novelti/StartExperiment.h>
#include <novelti/Command.h>
#include <novelti/common.cpp>
#include <novelti/inference_unit.h>
#include <novelti/position_control.h>
#include <novelti/orientation_disk_divider.h>
#include <novelti/opt_orientation_selector.h>

namespace novelti {


class OrientationControl : public InferenceUnit {
public:
    const float PDF_UNREACHABLE = -10.0;
    
    enum State { RELAXING, INFERRING_ORIENTATION, INFERRED};
    State state;
    
    ros::NodeHandle&    node_;
    ros::Publisher      pub_opdf_;
    ros::Publisher      pub_pose_inf_;
    
    OrientationPdf  opdf;
    std::vector<int> orientation_divided;
    geometry_msgs::PoseStamped pose_inferred;
    
    std::vector<double> interface_matrix;
    std::vector<double> priors;
    std::vector<double> posteriors;
    std::vector<double> coefs;
    
    float uniform_prob_;
    float eps_;
    double max_oprob_;
    int max_oprob_k_;
    float orientation_resol_;
    float orientation_inferred_;
    DiskDivider* disk_div_;
    OptOrientationSelector* orien_sel_;
    OrientationPdf opdf_;
    const PositionControl& pc_; 
    
    OrientationControl(ros::NodeHandle& node, const PositionControl& position_control);
    //InferenceUnit(const std::string paramPrefix);
    void start(novelti::StartExperiment::Request& req);
    void stop();
    const std::vector<int>& getOrientationColor();
    void initPriors(std::vector<double>& new_priors);
    void update(const std::vector<double>& coefs, const int cmd, std::vector<double>& priors);
    void onInferred(int inferredCmd);
    void act();
    void pubOpdf();

    
};
}

#endif