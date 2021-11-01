#ifndef OPT_ORIENTATION_SELECTOR_H
#define OPT_ORIENTATION_SELECTOR_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <novelti/OrientationPdf.h>
#include <tf/tf.h>
#include <string>

namespace novelti {

class OptOrientationSelector{
public:
    ros::NodeHandle node;
    ros::Publisher pub_opt_orientation;
    
    OrientationPdf const * opdf_;
    geometry_msgs::PoseStamped pose_desired_;
    int min_index;
    float resol;
    std::string method;
    

    OptOrientationSelector();
    void start();
    void setPositionInferred(const geometry_msgs::PoseStamped&);
    void findOptOrientation(const OrientationPdf& opdf);
    void selectOptIntermediateOrientation();
    float calculateProbCost(int index);
    void publishOrientation();
};

}

#endif 