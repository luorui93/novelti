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

OrientationPdfConstPtr orientationPdfToPtr (OrientationPdf pdf) {
    OrientationPdfConstPtr ptr(new OrientationPdf(pdf));
    return ptr;
}

void OrientationControl::start(novelti::StartExperiment::Request& req) {
    opdf_ = OrientationPdf();
    opdf_.header.frame_id = "/orientation_display";
    opdf_.data = std::vector<float>(360 / orientation_resol_,0.0);

    pub_pose_inf_ = node_.advertise<geometry_msgs::PoseStamped>("/pose_inferred", 1, true);
    pub_opdf_     = node_.advertise<OrientationPdf>("/opdf", 1, true);

    disk_div_->initDisplay(orientationPdfToPtr(opdf_));
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
    float uniform_prob = 1.0 / opdf_.data.size(); 
    PdfUtils::setUniform(opdf_.data, uniform_prob_);
    pubOpdf();
    disk_div_->orientationPdfCallback(orientationPdfToPtr(opdf_));

    //Calculate new prioris based on current opdf
    PdfUtils::accumulate<float>(opdf_.data, getOrientationColor(), new_priors);

}

void OrientationControl::update(const std::vector<double>& coefs, const int cmd, std::vector<double>& priors) {
    disk_div_->highlightSelection(cmd);
    PdfUtils::update<float>(opdf.data, getOrientationColor(),coefs);
    PdfUtils::denullifyNormalize<float>(opdf.data, eps_);

    PdfUtils::accumulate<float>(opdf.data, getOrientationColor(), priors);
}

void OrientationControl::onInferred(int inferredCmd) {
    orientation_inferred_ = max_oprob_k_ * orientation_resol_ * M_PI / 180;
    ROS_WARN("Orientation Inferred: %f", orientation_inferred_);
    pose_inferred = pc_.getPositionInferred();
    pose_inferred.pose.orientation = tf::createQuaternionMsgFromYaw(orientation_inferred_);
    pub_pose_inf_.publish(pose_inferred);
}

void OrientationControl::act() {
    pubOpdf();
    
    orien_sel_->findOptOrientation(orientationPdfToPtr(opdf));
    disk_div_->orientationPdfCallback(orientationPdfToPtr(opdf));
}

void OrientationControl::pubOpdf() {
    opdf_.header.stamp = ros::Time::now();
    pub_opdf_.publish(opdf);
    ros::spinOnce();
    //ROS_INFO("%s: =========== published opdf (SEQ=%d), max_oprob=%f", getName().c_str(), opdf.header.seq, max_oprob);
    opdf_.header.seq++;
}
