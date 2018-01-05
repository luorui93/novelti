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
    pub_opt_orientation = node.advertise<geometry_msgs::PoseStamped>("/orientation_desired",1,true);
}

void OptOrientationSelector::orientationPdfCallback(OrientationPdfConstPtr opdf_ptr, geometry_msgs::PoseStampedConstPtr pose_ptr) {
    if (method == "opt") {
        opdf = opdf_ptr;
        pose_inferred = pose_ptr;
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
    for(int i = 0;i < opdf->data.size();i++) {
        cur = calculateProbCost(i);
        if (cur < min) {
            min = cur;
            min_index = i;
        }
    }
}

float OptOrientationSelector::calculateProbCost(int index) {
    int size = opdf->data.size();
    float cost = 0.0;
    int angular_diff = 0;
    for (int i = 0;i < size;i++) {
        angular_diff = (i <= size/2) ? i : size - i;
        cost += opdf->data[(index+i)%size] * angular_diff;
    }
    return cost;
}

void OptOrientationSelector::publishOrientation() {
    geometry_msgs::PoseStamped opt_orientation;

    opt_orientation.pose.position = pose_inferred->pose.position;
    opt_orientation.pose.orientation = tf::createQuaternionMsgFromYaw(min_index*resol*M_PI/180.0);
    pub_opt_orientation.publish(opt_orientation);
}