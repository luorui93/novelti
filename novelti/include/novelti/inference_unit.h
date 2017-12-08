#ifndef NOVELTI_INFERENCE_UNIT_H
#define NOVELTI_INFERENCE_UNIT_H
/*
       subs                                  pubs
                   +------------------+
                   |                  |
/cmd_detected ---> |                  |
 /map_divided ---> |  inference_unit  | ---> /pdf
        /task ---> |                  |
                   |                  |
                   +------------------+
                             ^
                             |
                       srv: start
                           req:  scene
                           resp: -
*/

#include <vector>
#include <string>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>

#include <novelti/IntMap.h>
#include <novelti/FloatMap.h>
#include <novelti/OrientationPdf.h>
#include <novelti/StartExperiment.h>
#include <novelti/Command.h>
#include <novelti/common.cpp>


namespace novelti {

class InferenceUnit{
public:
    const float PDF_UNREACHABLE = -10.0;
    
    enum State { DEINFERENCE, INFERRING_POSITION, INFERRING_ORIENTATION, INFERRED};
    enum FastState { RCVD_NONE, RCVD_CMD, RCVD_MAPDIV };
    State state;
    FastState fast_state;
    
    ros::NodeHandle node;
    ros::Publisher      pub_pdf;
    ros::Publisher      pub_opdf;
    ros::Publisher      pub_pose_inf;
    ros::Publisher      pub_position_inf;
    ros::Publisher      pub_norm_pdf;
    ros::Subscriber     sub_map_div;
    ros::Subscriber     sub_cmd;
    ros::ServiceServer  srv_new_goal;
    
    FloatMap        pdf;
    OrientationPdf  opdf;
    FloatMap        norm_pdf;
    IntMapConstPtr  map_divided;
    std::vector<int> orientation_divided;
    CommandConstPtr cmd_detected;
    geometry_msgs::PoseStamped pose_inferred;
    
    std::vector<double> interface_matrix;
    std::vector<float> pois_;
    std::vector<double> priors;
    std::vector<double> posteriors;
    std::vector<double> coefs;

    std::vector<int> view_sizes_;
    int view_size_id_;
    std::vector<int> smooth_rads_;
    
    int n_cmds;
    float thresh_high;
    float thresh_low;
    float uniform_prob_;
    bool reset_pdf_on_new_;
    bool new_pdf_;
    float eps;
    double max_prob;
    double min_prob;
    double max_oprob;
    int max_prob_k;
    int max_oprob_k;
    float interest_area_thresh_;
    bool isNode;
    bool check_sync_;
    float orientation_resol;
    float orientation_inferred;
    
    InferenceUnit();
    InferenceUnit(const std::string paramPrefix);
    bool srvNewGoal(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp);
    void stopExp();
    void startExp(novelti::StartExperiment::Request& req);
    void resetPdf();
    void setUniformPdf();
    void setUniformOrientationPdf();
    void setStaticPredictedPdf();
    void mapDivCallback(novelti::IntMapConstPtr msg);
    void cmdCallback(CommandConstPtr cmd);
    void noveltiInfCallback(novelti::IntMapConstPtr ptr_map, CommandConstPtr ptr_cmd);
    void orientationInfCallback(std::vector<int>& unit_color, CommandConstPtr ptr_cmd);
    void updatePdfAndPublish();
    void updateOrientationPdfAndPublish();
    bool doesNeedSmoothing(int cx, int cy, int smooth_rad);
    void smoothenPdf();
    void publishViewTf();
    void calcPriors();
    void pubPdf();
    void pubPositionInferred(int k);
    void normalizePdf();
    
    void updatePdf();
    void denullifyPdf(std::vector<float>& pdf);
    void updateInferenceState();

    
    void calcUpdCoefs();
};
}

#endif