#ifndef NOVELTI_SHARED_CONTROL_H
#define NOVELTI_SHARED_CONTROL_H

#include <ros/ros.h>

#include <novelti/inference_unit.h>
#include <novelti/best_pose_finder.h>
#include <novelti/map_divider.h>

using namespace cwave;

namespace novelti {

class NoveltiSharedControl : public SynchronizableNode {
public:
    InferenceUnit* iu;
    BestPoseFinder* bpf;
    MapDivider* mdiv;
    std::mutex inference_started_lock;

    ros::Subscriber     sub_cmd;
    ros::ServiceServer  srv_new_goal;

    std::string divMethod;
    std::string posMethod;
    bool inference_started;

    NoveltiSharedControl(std::string divMethod, std::string posMethod);
    bool srvNewGoal(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp);
    void start(novelti::StartExperiment::Request& req);
    void stop();
    void cmdCallback(CommandConstPtr cmd);
    bool isInferenceStarted();
    void setInferenceStarted(bool value);
};

}


#endif