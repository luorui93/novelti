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
    
    OrientationPdfConstPtr opdf;
    geometry_msgs::PoseStampedConstPtr pose_inferred;
    int min_index;
    float resol;
    std::string method;
    

    OptOrientationSelector();
    void start();
    void findOptOrientation(OrientationPdfConstPtr opdf_ptr);
    void selectOptIntermediateOrientation();
    float calculateProbCost(int index);
    void publishOrientation();
};

}

#endif 