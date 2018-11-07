#include <novelti/novelti_shared_control.h>
#include <novelti/pdf_utils.h>
#include <novelti/position_control.h>
#include <novelti/orientation_control.h>

using namespace novelti;

NoveltiSharedControl::NoveltiSharedControl():
    SynchronizableNode()
{
    std::vector<double> mx;
    node_.getParam("interface_matrix", mx);
    node_.getParam("thresh_inferred", thresh_inferred_);
    node_.getParam("thresh_relaxed",  thresh_relaxed_);
    inf_mx_     = new InferenceMatrix(mx);
    priors_.resize(inf_mx_->nCmds_, 1.0/(inf_mx_->nCmds_));
    coefs_      = vector<double>(inf_mx_->nCmds_);
    PositionControl* position_control = new PositionControl(node_);
    units_.push_back(new ActionControl());
    units_.push_back(position_control);
    units_.push_back(new OrientationControl(node_, inf_mx_->nCmds_, *position_control));
    names_ = {"Action Control", "Position Control", "Orientation Control"};
    transitionMx_ = {
        std::vector<int>(inf_mx_->nCmds_, 1),
        std::vector<int>(inf_mx_->nCmds_, 2),
        std::vector<int>(inf_mx_->nCmds_, 0),
    };
}

NoveltiSharedControl::~NoveltiSharedControl() {
    delete inf_mx_;
    for (auto& ref: units_)
        delete ref;
}

void NoveltiSharedControl::start(StartExperiment::Request& req) {
    ROS_INFO("STARTING...");
    for (auto& p: priors_)
        p = 1.0/priors_.size();
    for (auto& ref: units_)
        ref->start(req);
    cur_ = 0;
    startNewInference();
    sub_cmd_ = node_.subscribe("/cmd_detected", 1, &NoveltiSharedControl::cmdCallback, this);    
}

void NoveltiSharedControl::stop() {
    ROS_INFO("STOPPING...");
    for (auto& ref: units_)
        ref->stop();
    sub_cmd_.shutdown();
}

void NoveltiSharedControl::startNewInference() {
    ROS_INFO(">>>>>>>>>>>>>>>>>> Starting new inference: %s<<<<<<<<<<<<<<<<<<<", names_[cur_].c_str());
    units_[cur_]->initPriors(priors_);
    units_[cur_]->act();
    PdfStats<double> stats(priors_);
    relaxing_ = (stats.max > thresh_inferred_);
    ROS_INFO("%s: initialized probabilities: %s, relaxing=%d", 
        getName().c_str(),                        
        InferenceMatrix::toString(priors_).c_str(),
        relaxing_);
}

void NoveltiSharedControl::cmdCallback(CommandConstPtr cmd) {
    ROS_INFO("Detected command: %d", cmd->cmd);
    inf_mx_->calcUpdateCoefs(priors_, cmd->cmd, coefs_);
    units_[cur_]->update(coefs_, cmd->cmd, priors_);
    ROS_INFO("%s: current probabilities after update: %s", 
             getName().c_str(),
             InferenceMatrix::toString(priors_).c_str());
    PdfStats<double> stats(priors_);
    if (relaxing_) { 
        if (stats.max < thresh_relaxed_) {
            relaxing_ = false;
            ROS_INFO("relaxing -> inferring");
        }
    } else {
        if (stats.max > thresh_inferred_) {
            ROS_INFO("inferring -> inferred, starting new inference");
            units_[cur_]->onInferred(stats.max_k);
            cur_ = transitionMx_[cur_][stats.max_k];
            startNewInference();
            return;
        }
    }
    units_[cur_]->act();
}



int main(int argc, char **argv) {
    ros::init(argc, argv, "novelti_shared_control");
    NoveltiSharedControl nsc;
    nsc.run();
}