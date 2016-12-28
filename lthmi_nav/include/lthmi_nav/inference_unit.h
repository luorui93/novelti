#ifndef LTHMI_NAV_INFERENCE_UNIT_H
#define LTHMI_NAV_INFERENCE_UNIT_H
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

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseStamped.h>

#include <lthmi_nav/IntMap.h>
#include <lthmi_nav/FloatMap.h>
#include <lthmi_nav/StartExperiment.h>
#include <lthmi_nav/Command.h>
#include <lthmi_nav/common.cpp>


namespace lthmi_nav {

class InferenceUnit : public SynchronizableNode {
public:
    const float PDF_UNREACHABLE = -10.0;
    
    enum State { INFERRING, INFERRED, INFERRING_NEW };
    enum FastState { RCVD_NONE, RCVD_CMD, RCVD_MAPDIV };
    State state;
    FastState fast_state;
    
    ros::Publisher      pub_pdf;
    ros::Publisher      pub_pose_inf;
    ros::Subscriber     sub_map_div;
    ros::Subscriber     sub_cmd;
    ros::ServiceServer  srv_new_goal;
    
    FloatMap        pdf;
    IntMapConstPtr  map_divided;
    CommandConstPtr cmd_detected;
    
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
    double eps;
    double max_prob;
    int max_prob_k;
    float interest_area_thresh_;
    
    InferenceUnit();
    bool srvNewGoal(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp);
    void stop();
    void start(lthmi_nav::StartExperiment::Request& req);
    void resetPdf();
    void setUniformPdf();
    void setHardcodedPredictedPdf();
    void mapDivCallback(lthmi_nav::IntMapConstPtr msg);
    void cmdCallback(CommandConstPtr cmd);
    void updatePdfAndPublish();
    bool doesNeedSmoothing(int cx, int cy, int smooth_rad);
    void smoothenPdf();
    void publishViewTf();
    void calcPriors();
    void pubPdf();
    void pubPoseInferred(int k);
    
    void updatePdf();
    void denullifyPdf();

    
    void calcUpdCoefs();
};
}

#endif