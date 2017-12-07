#ifndef ORIENTATION_CONTROL_H
#define ORIENTATION_CONTROL_H

#include <ros/ros.h>
#include <novelti/IntMap.h>
#include <novelti/OrientationPdf.h>
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

class OrientationControl {
public:

    OrientationControl();
    void initDisplay(OrientationPdfConstPtr opdf);
    void orientationPdfCallback(OrientationPdfConstPtr pdf);
    void setColor(int x,int y,int color);
    void updateOptimalPdf(int cur_color);
    void markUnitColor();
    void updateArc();
    EndPoint calculateEndPoint(float);
    void divideAndPublish();
    void drawSector();
    void resetArc();

    ros::NodeHandle node;
    ros::Publisher pub_canvas;

    IntMap canvas;
    novelti::OrientationPdfConstPtr opdf_ptr;
    double radius;
    int n_cmd;
    int color_order;
    double resol,x_center,y_center;

    std::vector<float> cur_color_pdf;
    std::vector<float> optimal_color_pdf;
    std::vector<int> unit_color;
    std::vector<Arc> arc_vector;
};

}
#endif