#include <novelti/orientation_disk_divider.h>
#include "SectorWalker.hpp"
#include <SectorArcs.hpp>
#include <ArcVertices.hpp>
#include <math.h>

using namespace novelti;

DiskDivider::DiskDivider(int n_cmd):
    node("~"),
    n_cmd_(n_cmd)
{
    node.param("ori/compass_radius", radius, 100.0);
    node.param("ori/command_number", n_cmd, 4);
    node.param("ori/orientation_resolution", resol, 5.0);

} 

void DiskDivider::initDisplay(OrientationPdfConstPtr opdf) {
    disk = IntMap();
    disk.header.frame_id = "/map";
    disk.info.width = 2*radius + 3;
    disk.info.height = 2*radius + 3;
    disk.info.resolution = 0.1;
    disk.info.origin.position.x = 0.0;
    disk.info.origin.position.y = 0.0;
    disk.data = std::vector<int>(disk.info.width*disk.info.height, 255);
    pub_disk = node.advertise<novelti::IntMap>("/orientation_divided", 1, true);
    pub_selection_highlight = node.advertise<novelti::IntMap>("/highlight_orientation",1,false);
    selection_highlight = disk;
    transparent_disk = disk;

    unit_color = std::vector<int>(opdf->data.size(),-1);
    arc_vector = std::vector<novelti::Arc>(n_cmd_);

    optimal_color_pdf = std::vector<float>(n_cmd_,1.0/n_cmd_);
    cur_color_pdf = std::vector<float>(optimal_color_pdf.size(),0.0);

    x_center = radius;
    y_center = radius;

    opdf_ptr = opdf;
    markUnitColor();
    updateArc();
    drawSector();
    resetDisk();
}   

void DiskDivider::resetDisk() {
    pub_disk.publish(transparent_disk);
}

void DiskDivider::orientationPdfCallback(OrientationPdfConstPtr opdf) {
    opdf_ptr = opdf;
    divideAndPublish();
}   

void DiskDivider::highlightSelection(const int& cmd){

    selection_highlight.data = disk.data;
    for (std::vector<int>::iterator it=selection_highlight.data.begin(); it != selection_highlight.data.end(); it++) {
        if (*it == cmd)
            continue;
        else *it = 255;
    }
    pub_selection_highlight.publish(selection_highlight);
    usleep(0.2*1000000);
    pub_selection_highlight.publish(transparent_disk);
}

void DiskDivider::setColor(int x, int y, int color) {
    disk.data[x + y*disk.info.width] = color;
}
void DiskDivider::updateOptimalPdf(int cur_color) {  //Try to divide region as balanced as possible based on optimal pdf
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

void DiskDivider::markUnitColor() {   //update color for minimal unit (resolution)
    int cur_color = 0;
    std::fill(cur_color_pdf.begin(),cur_color_pdf.end(),0.0);
    for (int i=0; i<opdf_ptr->data.size();i++) {
        cur_color_pdf[cur_color] += opdf_ptr->data[i];
        if (cur_color_pdf[cur_color] > optimal_color_pdf[cur_color] + std::numeric_limits<float>::epsilon() && cur_color < n_cmd_ - 1) {  //TODO: a better way to add epsilon

            cur_color_pdf[cur_color] -= opdf_ptr->data[i];
            updateOptimalPdf(cur_color);
            //ROS_WARN("turning point: %d",i);
            cur_color++;
            cur_color_pdf[cur_color] += opdf_ptr->data[i];
        }
        
        unit_color[i] = cur_color;
    }   
}

void DiskDivider::resetArc() {
    for (int i=0;i<arc_vector.size();i++) {
        arc_vector[i].color = 0;
        arc_vector[i].upper_angle = 0.0;
        arc_vector[i].lower_angle = 0.0;
    }
}

void DiskDivider::updateArc() {   //update arc color
    int cur_color = 0;
    resetArc();
    for (int i=0;i<unit_color.size();i++) {
        if (unit_color[i] == cur_color) {
            arc_vector[cur_color].color = cur_color;
            arc_vector[cur_color].upper_angle += resol * M_PI/180;
        }
        else if (cur_color < n_cmd_ - 1) {
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

EndPoint DiskDivider::calculateEndPoint(float angle) {   //calculate arc endpoint
    EndPoint p;
    p.quadrant = floor(2*angle / M_PI);
    angle = angle - p.quadrant * M_PI/2;
    p.x = floor(radius * cos(angle));
    p.y = floor(radius * sin(angle));
}

void DiskDivider::drawSector() {
    std::fill(disk.data.begin(),disk.data.end(),255);
    for (int i=0; i < arc_vector.size();i++) {
        if (!(arc_vector[i].lower_angle == arc_vector[i].upper_angle) ){
        // ROS_WARN("----------------------Paint sector %d", i);
//             for (auto pt: cwave::SectorPoints(cwave::Point(x_center,y_center), arc_vector[i].lower_angle, arc_vector[i].upper_angle, radius,10000))
// -                setColor(pt.x,pt.y,arc_vector[i].color);

            cwave::SectorArcs sectorArcs(arc_vector[i].lower_angle, arc_vector[i].upper_angle, radius, 10000);
            for (const auto& arc: sectorArcs)
                for (const auto& vx: cwave::ArcVertices(arc))
                    setColor(vx.x,vx.y,arc_vector[i].color);
        }
    }
}    

void DiskDivider::divideAndPublish() {
    markUnitColor();
    updateArc();
    drawSector();
    pub_disk.publish(disk);
}