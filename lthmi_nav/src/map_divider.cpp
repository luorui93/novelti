//#define DEBUG_DIVIDER 1

/*
       subs                                       pubs
                   +--------------------+
                   |                    | ---> /map_divided
/pose_optimal ---> |                    | 
         /pdf ---> |                    | ---> /debug_pose         |
                   |  node_map_divider  | ---> /debug_pose_border  |
                   |                    | ---> /debug_map_track    | debug
                   |                    | ---> /debug_map_dist     |
                   |                    | ---> /debug_stars        |
                   +--------------------+
                              ^
                              |
                        srv: start
                            req:  scene
                            resp: -
*/

#include <lthmi_nav/map_divider.h>


using namespace lthmi_nav;

MapDivider::MapDivider() :
        SynchronizableNode()
{
    node.getParam("probs_optimal", probs_optimal);
}
    
void MapDivider::stop() {
    sub_pose_opt.shutdown();
    sub_pdf.shutdown();
    pub_map_div.shutdown();
}

void MapDivider::start(lthmi_nav::StartExperiment::Request& req) {
    state = WAITING;
    map_divided.info.resolution = req.map.info.resolution;
    map_divided.info.width = req.map.info.width;
    map_divided.info.height = req.map.info.height;
    map_divided.data = std::vector<int>(req.map.data.size(), 0);
    sub_pose_opt  = node.subscribe("/pose_optimal", 1, &MapDivider::poseOptCallback, this);
    sub_pdf       = node.subscribe("/pdf", 1, &MapDivider::pdfCallback, this);
}

void MapDivider::poseOptCallback(const geometry_msgs::PoseStamped& pose) {
    ROS_INFO_NAMED(getNamespace(), "received pose");
    ///vx(pose, 0.1);//(double)(pdf->info.resolution));
    if (state==ONLY_PDF) {
        divideAndPublish();
        state = WAITING;
    } else {
        state = ONLY_POSE;
    }
}

void MapDivider::pdfCallback(lthmi_nav::FloatMapConstPtr msg){
    ROS_INFO_NAMED(getNamespace(), "received pdf");
    pdf = msg;
    if (state==ONLY_POSE) {
        divideAndPublish();
        state = WAITING;
    } else {
        state = ONLY_PDF;
    }
}

void MapDivider::divideAndPublish() {
    ROS_INFO_NAMED(getNamespace(), "starting to divide");
    //divide();
    //pub_map_div.publish(map_divided);
}

