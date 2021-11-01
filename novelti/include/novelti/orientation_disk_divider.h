#ifndef ORIENTATION_DISK_DIVIDER_H
#define ORIENTATION_DISK_DIVIDER_H

#include <ros/ros.h>
#include <novelti/IntMap.h>
#include <novelti/OrientationPdf.h>
#include <geometry_msgs/PoseStamped.h>
#include <novelti/Command.h>
#include <vector>

namespace novelti {

struct Arc {
    float lower_angle;
    float upper_angle;
    int color;
};

struct EndPoint {
    int x;
    int y;
    int quadrant;
};

class DiskDivider {
public:

    DiskDivider(int);
    void initDisplay(const OrientationPdf& opdf, const geometry_msgs::PoseStamped&);
    void orientationPdfCallback(const OrientationPdf& opdf);
    void setColor(int x,int y,int color);
    void updateOptimalPdf(int cur_color);
    void markUnitColor();
    void updateArc();
    EndPoint calculateEndPoint(float);
    void divideAndPublish();
    void drawSector();
    void resetArc();
    void highlightSelection(const int& cmd);
    void resetDisk();
    
    ros::NodeHandle node;
    ros::Publisher pub_disk,pub_selection_highlight;

    IntMap disk,selection_highlight,transparent_disk;
    novelti::OrientationPdf const * opdf_;
    double radius;
    int n_cmd_;
    int color_order;
    double resol,x_center,y_center;

    std::vector<double> cur_color_pdf;
    std::vector<double> optimal_color_pdf;
    std::vector<int> unit_color;
    std::vector<Arc> arc_vector;
};

}
#endif