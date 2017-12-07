#include <novelti/orientation_control.h>
#include "SectorWalker.hpp"
#include <math.h>

using namespace novelti;

OrientationControl::OrientationControl():
    node("~") 
{
    node.param("ori/compass_radius", radius, 100.0);
    node.param("ori/command_number", n_cmd, 4);
    node.param("ori/orientation_resolution", resol, 5.0);

} 

void OrientationControl::initDisplay(OrientationPdfConstPtr opdf) {
    canvas = IntMap();
    canvas.header.frame_id = "/map";
    canvas.info.width = 2*radius + 3;
    canvas.info.height = 2*radius + 3;
    canvas.info.resolution = 0.1;
    canvas.info.origin.position.x = 0.0;
    canvas.info.origin.position.y = 0.0;
    canvas.data = std::vector<int>(canvas.info.width*canvas.info.height, 255);
    pub_canvas = node.advertise<novelti::IntMap>("/orientation_divided", 1, true);

    unit_color = std::vector<int>(opdf->data.size(),-1);
    arc_vector = std::vector<novelti::Arc>(n_cmd);

    optimal_color_pdf = std::vector<float>(n_cmd,1.0/n_cmd);
    cur_color_pdf = std::vector<float>(optimal_color_pdf.size(),0.0);

    x_center = radius;
    y_center = radius;

    opdf_ptr = opdf;
    markUnitColor();
    updateArc();
    drawSector();
}   

void OrientationControl::orientationPdfCallback(OrientationPdfConstPtr opdf) {
    opdf_ptr = opdf;
    divideAndPublish();
}   

void OrientationControl::setColor(int x, int y, int color) {
    canvas.data[x + y*canvas.info.width] = color;
}
void OrientationControl::updateOptimalPdf(int cur_color) {  //Try to divide region as balanced as possible based on optimal pdf
    float total_prob = 0.0;
    for (int i=0;i<=cur_color;i++) {
        total_prob += optimal_color_pdf[i];
    }
    //ROS_WARN("total_prob: %f",total_prob);
    for (int i=cur_color+1;i < optimal_color_pdf.size();i++) {
        optimal_color_pdf[i] = (1-total_prob) / (optimal_color_pdf.size()-cur_color-1);
    }
    //ROS_WARN("optimal_color_pdf: [%f,%f,%f,%f]",optimal_color_pdf[0],optimal_color_pdf[1],optimal_color_pdf[2],optimal_color_pdf[3]);

}

void OrientationControl::markUnitColor() {   //update color for minimal unit (resolution)
    int cur_color = 0;
    std::fill(cur_color_pdf.begin(),cur_color_pdf.end(),0.0);
    for (int i=0; i<opdf_ptr->data.size();i++) {
        cur_color_pdf[cur_color] += opdf_ptr->data[i];
        if (cur_color_pdf[cur_color] > optimal_color_pdf[cur_color] + std::numeric_limits<float>::epsilon() && cur_color < n_cmd - 1) {  //TODO: a better way to add epsilon

            cur_color_pdf[cur_color] -= opdf_ptr->data[i];
            updateOptimalPdf(cur_color);
            //ROS_WARN("turning point: %d",i);
            cur_color++;
            cur_color_pdf[cur_color] += opdf_ptr->data[i];
        }
        
        unit_color[i] = cur_color;
    }   
}

void OrientationControl::resetArc() {
    for (int i=0;i<arc_vector.size();i++) {
        arc_vector[i].color = 0;
        arc_vector[i].upper_angle = 0.0;
        arc_vector[i].lower_angle = 0.0;
    }
}

void OrientationControl::updateArc() {   //update arc color
    int cur_color = 0;
    resetArc();
    for (int i=0;i<unit_color.size();i++) {
        if (unit_color[i] == cur_color) {
            arc_vector[cur_color].color = cur_color;
            arc_vector[cur_color].upper_angle += resol * M_PI/180;
        }
        else if (cur_color < n_cmd - 1) {
            arc_vector[cur_color+1].lower_angle = arc_vector[cur_color].upper_angle;
            arc_vector[cur_color+1].upper_angle = arc_vector[cur_color+1].lower_angle + resol*M_PI/180;
            cur_color++;
            arc_vector[cur_color].color = cur_color;
        }
    }
    // for (int i=0;i<arc_vector.size();i++) {
    //     ROS_WARN("arc_vector:%f,%f,%d",arc_vector[i].upper_angle,arc_vector[i].lower_angle,i);
    // }
}

EndPoint OrientationControl::calculateEndPoint(float angle) {   //calculate arc endpoint
    EndPoint p;
    p.quadrant = floor(2*angle / M_PI);
    angle = angle - p.quadrant * M_PI/2;
    p.x = floor(radius * cos(angle));
    p.y = floor(radius * sin(angle));
}

void OrientationControl::drawSector() {
    std::fill(canvas.data.begin(),canvas.data.end(),255);
    for (int i=0; i < arc_vector.size();i++) {
        if (!(arc_vector[i].lower_angle == arc_vector[i].upper_angle) ){
        // ROS_WARN("----------------------Paint sector %d", i);
            for (auto pt: cwave::SectorPoints(cwave::Point(x_center,y_center), arc_vector[i].lower_angle, arc_vector[i].upper_angle, radius,10000)) {
                setColor(pt.x,pt.y,arc_vector[i].color);
            }    
        }
    }
}

void OrientationControl::divideAndPublish() {
    markUnitColor();
    updateArc();
    drawSector();
    pub_canvas.publish(canvas);
}