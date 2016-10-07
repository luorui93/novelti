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
    if (probs_optimal.size()==0)
        throw ros::Exception("ERROR: probs_optimal parameter should be specified and have length>0");
}
    
void MapDivider::stop() {
    sub_pose_opt.shutdown();
    sub_pdf.shutdown();
    pub_map_div.shutdown();
}

void MapDivider::start(lthmi_nav::StartExperiment::Request& req) {
    state = WAITING;
    probs_actual = std::vector<double>(probs_optimal.size(),0.0);
    map_divided.info.resolution = req.map.info.resolution;
    map_divided.info.width = req.map.info.width+1;
    map_divided.info.height = req.map.info.height+1;
    map_divided.info.origin.position.x = -0.5*req.map.info.resolution;
    map_divided.info.origin.position.y = -0.5*req.map.info.resolution;
    map_divided.data = std::vector<int>(map_divided.info.width*map_divided.info.height, 255);
    pub_map_div   = node.advertise<lthmi_nav::IntMap>("/map_divided", 1, false); //not latched
    sub_pose_opt  = node.subscribe("/pose_optimal", 1, &MapDivider::poseOptCallback, this);
    sub_pdf       = node.subscribe("/pdf", 1, &MapDivider::pdfCallback, this);
}

void MapDivider::poseOptCallback(const geometry_msgs::PoseStamped& pose) {
    ROS_INFO("%s: received pose", getName().c_str());
    //vx(pose, 0.1);//(double)(pdf->info.resolution));
    new (&vx) Vertex(pose, (double)(map_divided.info.resolution));
    if (state==ONLY_PDF) {
        divideAndPublish();
        state = WAITING;
    } else {
        state = ONLY_POSE;
    }
}

void MapDivider::pdfCallback(lthmi_nav::FloatMapConstPtr msg){
    ROS_INFO("%s: received pdf", getName().c_str());
    pdf = msg;
    if (state==ONLY_POSE) {
        divideAndPublish();
        state = WAITING;
    } else {
        state = ONLY_PDF;
    }
}

void MapDivider::divideAndPublish() {
    ROS_INFO("%s: starting to divide", getName().c_str());
    divide();
    pub_map_div.publish(map_divided);
    ROS_INFO("%s: published divided map", getName().c_str());
}

