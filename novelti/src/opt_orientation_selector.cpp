#include <novelti/opt_orientation_selector.h>
#include <math.h>

using namespace novelti;

OptOrientationSelector::OptOrientationSelector():
    node("~")
{
    node.param<float>("/orientation_resol",resol,5.0);
    node.param<std::string>("ori/method",method,"opt");
}

void OptOrientationSelector::start() {
    pub_opt_orientation = node.advertise<geometry_msgs::PoseStamped>("/orientation_desired",1,false);
}

void OptOrientationSelector::findOptOrientation(const OrientationPdf& opdf) {
    if (method == "opt") {
        opdf_ = &opdf;
        selectOptIntermediateOrientation();
        publishOrientation();
    }
    else if (method == "still") {
        return;
    }
}

void OptOrientationSelector::selectOptIntermediateOrientation() {
    float min = std::numeric_limits<float>::max();
    min_index = 0;
    int cur = 0;
    for(int i = 0;i < opdf_->data.size();i++) {
        cur = calculateProbCost(i);
        if (cur < min) {
            min = cur;
            min_index = i;
        }
    }
}

void OptOrientationSelector::setPositionInferred(const geometry_msgs::PoseStamped& position_inferred) {
    pose_desired_=position_inferred;
}


float OptOrientationSelector::calculateProbCost(int index) {
    int size = opdf_->data.size();
    float cost = 0.0;
    int angular_diff = 0;
    for (int i = 0;i < size;i++) {
        angular_diff = (i <= size/2) ? i : size - i;
        cost += opdf_->data[(index+i)%size] * angular_diff;
    }
    return cost;
}

void OptOrientationSelector::publishOrientation() {
    pose_desired_.header.stamp = ros::Time::now();
    pose_desired_.pose.orientation = tf::createQuaternionMsgFromYaw(min_index*resol*M_PI/180.0);
    pub_opt_orientation.publish(pose_desired_);
}