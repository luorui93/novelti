#ifndef NOVELTI_SHARED_CONTROL_H
#define NOVELTI_SHARED_CONTROL_H

#include <ros/ros.h>
#include <novelti/inference_unit.h>
#include <novelti/synchronizable_node.h>
#include <novelti/inference_matrix.h>
#include <novelti/pdf_utils.h>
#include <novelti/Command.h>
// #include <novelti/FloatMap.h>

namespace novelti {
  
class ActionControl : public InferenceUnit {
public:    
    ActionControl() {}
protected:    
    void start(StartExperiment::Request& req) {};
    void stop() {};
    void initPriors(std::vector<double>& priors) { 
        PdfUtils::setUniform(priors, 1.0/priors.size());
    }
    void update(const std::vector<double>& coefs, const int cmd, std::vector<double>& priors) {
        for (int k=0; k<coefs.size(); k++)
            priors[k] *= coefs[k];
    }
    void onInferred(int cmd) {};
    void act() {};
};    



class NoveltiSharedControl : public SynchronizableNode {
public:
    NoveltiSharedControl();
    ~NoveltiSharedControl();

    Command cmd_;
//     FloatMap dd;
protected:
    double thresh_inferred_;
    double thresh_relaxed_;
    InferenceMatrix*    inf_mx_;
    std::vector<double> probs_optimal_;
    std::vector<std::string> names_;
    std::vector<double> priors_;
    std::vector<double> coefs_;
    ros::Subscriber     sub_cmd_;
    std::vector<InferenceUnit*> units_;
    std::vector<std::vector<int>> transitionMx_;
    int cur_;
    bool relaxing_;
   
    void start(StartExperiment::Request& req);
    void stop(); 
    void cmdCallback(CommandConstPtr cmd);
    void startNewInference();
};

}

#endif