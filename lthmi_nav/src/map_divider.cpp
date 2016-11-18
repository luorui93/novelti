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
    #ifdef DEBUG_DIVIDER
        pub_debug_map_div.shutdown();
        pub_debug_pose.shutdown();
    #endif
}

void MapDivider::start(lthmi_nav::StartExperiment::Request& req) {
    state = WAITING;
    probs_actual = std::vector<double>(probs_optimal.size(),0.0);
    
    map_divided = IntMap();
    map_divided.header.frame_id = "/map";
    map_divided.info.resolution = req.map.info.resolution;
    map_divided.info.width = req.map.info.width+1;
    map_divided.info.height = req.map.info.height+1;
    map_divided.info.origin.position.x = -0.5*req.map.info.resolution;
    map_divided.info.origin.position.y = -0.5*req.map.info.resolution;
    map_divided.data = std::vector<int>(map_divided.info.width*map_divided.info.height, 255);
    
    pub_map_div   = node.advertise<lthmi_nav::IntMap>("/map_divided", 1, true); //latched
    sub_pose_opt  = node.subscribe("/pose_best", 1, &MapDivider::poseOptCallback, this);
    sub_pdf       = node.subscribe("/pdf", 1, &MapDivider::pdfCallback, this);
    
    #ifdef DEBUG_DIVIDER
        pub_debug_map_div   = node.advertise<lthmi_nav::IntMap>("/debug_map_divided", 1, true); //latched
        pub_debug_pose      = node.advertise<geometry_msgs::PoseStamped>("/debug_pose", 1, true); //not latched
    #endif
}

void MapDivider::poseOptCallback(geometry_msgs::PoseStampedConstPtr msg) {
    ROS_INFO("%s: received pose (SEQ==%d)", getName().c_str(), msg->header.seq);
    //vx(pose, 0.1);//(double)(pdf->info.resolution));
    pose_best = msg;
    if (state==ONLY_PDF) {
        state = WAITING;
        divideAndPublish();
        
    } else {
        state = ONLY_POSE;
    }
}

void MapDivider::pdfCallback(lthmi_nav::FloatMapConstPtr msg){
    ROS_INFO("%s: received pdf (SEQ==%d)", getName().c_str(), msg->header.seq);
    pdf = msg;
    if (state==ONLY_POSE) {
        state = WAITING;
        divideAndPublish();
    } else {
        state = ONLY_PDF;
    }
}

void MapDivider::divideAndPublish() {
    if (false && (map_divided.header.seq != pdf->header.seq || map_divided.header.seq != pose_best->header.seq)) {
        ROS_FATAL("%s: SYNCHRONIZATION BROKEN! map_divided.seq==%d, pdf.seq==%d, pose_best.seq==%d.",
                 getName().c_str(), map_divided.header.seq, pdf->header.seq, pose_best->header.seq);
        ros::shutdown();
        exit(1);
    }
    updateVertex(pose_best->pose, pt_best.x, pt_best.y);
    ROS_INFO("%s: starting to divide", getName().c_str());
    startDivider();
    divide();
    endDivider();
    ROS_INFO("%s: probs_actual: [%f, %f, %f, %f]", getName().c_str(), probs_actual[0], probs_actual[1], probs_actual[2], probs_actual[3]);
    map_divided.header.stamp = ros::Time::now();
    pub_map_div.publish(map_divided);
    ros::spinOnce();
    ROS_INFO("%s: published divided map", getName().c_str());
    map_divided.header.seq++;
}




void MapDivider::startDivider() {
    prob = 0.0;
    cur_region = 0;
    std::fill(probs_actual.begin(), probs_actual.end(), 0.0); //probs_actual = 0-vector
    probs_scaled = probs_optimal;
}

void MapDivider::updateProbsScaled() {
    double sum_optimal = 0.0;
    double sum_actual  = 0.0;
    for (int k=0; k<=cur_region; k++) {
        sum_optimal += probs_optimal[k];
        sum_actual  += probs_actual[k];
    }
    for (int k=cur_region+1; k<probs_optimal.size(); k++)
        probs_scaled[k] = probs_optimal[k]*(1.0-sum_actual)/(1.0-sum_optimal);
}

void MapDivider::markVertex(int x, int y) {
    markVertex(x + y*(pdf->info.width));
}

void MapDivider::markVertex(int k) {
    int cur = map_divided.data[k];
    double p = pdf->data[k];
    if (p>=0.0 && cur==255) {
        prob += p;
        if (prob > probs_scaled[cur_region] && cur_region<probs_scaled.size()-1) {
            //ROS_INFO("_________________ probs_actual = [%f,%f,%f,%f]", probs_actual[0], probs_actual[1], probs_actual[2], probs_actual[3]);
            //ROS_INFO("_________________ cur_region=%d, p=%f, prob=%f, probs_scaled[cur_region]=%f",cur_region, p, prob, probs_scaled[cur_region]);
            probs_actual[cur_region] = prob-p;
            updateProbsScaled();
            prob = p;
            cur_region++;

        }
        map_divided.data[k] = cur_region;
    }
    
}

void MapDivider::endDivider() {
    probs_actual[cur_region] = prob;
}

#ifdef DEBUG_DIVIDER
    void MapDivider::publishDebugPose(int x, int y) {
        //ROS_WARN("Debug pose vertex: (%d,%d)", x, y);
        geometry_msgs::PoseStamped msg;
        updatePose(msg, x, y);
        msg.header.frame_id="/map";
        pub_debug_pose.publish(msg);
    }
#endif
