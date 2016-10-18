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
    State state;
    ros::Publisher  pub_pdf;
    ros::Publisher  pub_pose_inf;
    ros::Subscriber sub_map_div;
    ros::Subscriber sub_cmd;
    ros::ServiceServer srv_new_goal;
    
    lthmi_nav::FloatMap pdf;
    lthmi_nav::IntMapConstPtr map_divided;
    
    std::vector<double> interface_matrix;
    std::vector<double> priors;
    std::vector<double> posteriors;
    std::vector<double> coefs;

    int n_cmds;
    float thresh_high;
    float thresh_low;
    float eps;
    double max_prob;
    int max_prob_k;
    
    InferenceUnit();
    bool srvNewGoal(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp);
    void stop();
    void start(lthmi_nav::StartExperiment::Request& req);
    void mapDivCallback(lthmi_nav::IntMapConstPtr msg);
    void cmdCallback(CommandConstPtr cmd);
    void pubPdf();
    void pubPoseInferred(int k);
    
    void updatePdf(int cmd_detected);
    void denullifyPdf();
    void calcPriors();
    void calcUpdCoefs(int cmd_detected);
};
}

#endif