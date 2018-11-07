#include <novelti/orientation_control.h>

using namespace novelti;

OrientationControl::OrientationControl(ros::NodeHandle& node, int n_cmds, const PositionControl& position_control) :
    node_(node),
    pc_(position_control)
{
    node_.param<float>("ori/orientation_resolution", orientation_resol_, 5.0);
    node_.param<float>("inf/eps", eps_, std::numeric_limits<float>::epsilon());

    disk_div_ = new DiskDivider(n_cmds);
    orien_sel_ = new OptOrientationSelector();
}

void OrientationControl::start(novelti::StartExperiment::Request& req) {
    opdf_ = OrientationPdf();
    opdf_.header.frame_id = "/orientation_display";
    opdf_.data = std::vector<float>(360 / orientation_resol_,0.0);

    pub_pose_inf_ = node_.advertise<geometry_msgs::PoseStamped>("/pose_inferred", 1, true);
    pub_opdf_     = node_.advertise<OrientationPdf>("/opdf", 1, true);

    disk_div_->initDisplay(opdf_);
    ROS_INFO("Orientation Control Started...");
    orien_sel_->start();
    ROS_INFO("Opt Orientation Selector Started...");
}

void OrientationControl::stop() {
    pub_pose_inf_.shutdown();
    pub_opdf_.shutdown();
}

const std::vector<int>& OrientationControl::getOrientationColor() {
    return disk_div_->unit_color;
}

void OrientationControl::initPriors(std::vector<double>& new_priors) {
    orien_sel_->setPositionInferred(pc_.getPositionInferred());
    uniform_prob_ = 1.0 / opdf_.data.size(); 
    PdfUtils::setUniform(opdf_.data, uniform_prob_);
    pubOpdf();
    disk_div_->orientationPdfCallback(opdf_);

    //Calculate new prioris based on current opdf
    stats_ = PdfUtils::accumulate<float>(opdf_.data, getOrientationColor(), new_priors);

}

void OrientationControl::update(const std::vector<double>& coefs, const int cmd, std::vector<double>& priors) {
    disk_div_->highlightSelection(cmd);
    PdfUtils::update<float>(opdf_.data, getOrientationColor(),coefs);
    PdfUtils::denullifyNormalize<float>(opdf_.data, eps_);

    orien_sel_->findOptOrientation(opdf_);
    disk_div_->orientationPdfCallback(opdf_);
    stats_ = PdfUtils::accumulate<float>(opdf_.data, getOrientationColor(), priors);

}

void OrientationControl::onInferred(int inferredCmd) {
    orientation_inferred_ = stats_.max_k * orientation_resol_ * M_PI / 180;
    ROS_WARN("Orientation Inferred: %f", orientation_inferred_);
    pose_inferred_ = pc_.getPositionInferred();
    pose_inferred_.pose.orientation = tf::createQuaternionMsgFromYaw(orientation_inferred_);
    pub_pose_inf_.publish(pose_inferred_);
}

void OrientationControl::act() {    
    
    pubOpdf();
}

void OrientationControl::pubOpdf() {
    opdf_.header.stamp = ros::Time::now();
    pub_opdf_.publish(opdf_);
    ros::spinOnce();
    //ROS_INFO("%s: =========== published opdf (SEQ=%d), max_oprob=%f", getName().c_str(), opdf.header.seq, max_oprob);
    opdf_.header.seq++;
}
