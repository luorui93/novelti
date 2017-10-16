/*
            subs                                       pubs
                    +-------------------------+
                    |                         |
          /pdf ---> |                         | ---> /pose_best
                    |  node_best_pose_finder  |
 /pose_current ---> |                         | ---> /reach_area   | debug
                    |                         |
                    +-------------------------+
                                ^
                                |
                            srv: start
                                req:  scene
                                resp: -
*/

#include <novelti/best_pose_finder.h>
#include <limits>


using namespace novelti;

BestPoseFinder::BestPoseFinder(const std::string paramPrefix):
    node("~")
{
    isNode = false;
    double max_vel, period, safety_coef;
    node.param<double>("max_vel", max_vel, 0.0);
    node.param<double>("safety_coef", safety_coef, 0.0);
    node.param<double>("period", period, 0.0);
    node.param<int>("pose_to_vertex_tolerance", pose_to_vertex_tolerance, 2);
    max_dist_float = max_vel*safety_coef*period;
    if (max_dist_float==0.0)
        throw ros::Exception("ERROR: At least one of the following node parameters (max_vel, period, safety_coef) is not specified or 0.0. All must be greater than zero.");
}

BestPoseFinder::BestPoseFinder():
    BestPoseFinder("")
{
    isNode = true;
}
    
void BestPoseFinder::stopExp() {
    sub_pose_cur.shutdown();
    sub_pdf.shutdown();
    pub_pose_best.shutdown();
    #ifdef DEBUG_POSE_FINDER
        pub_reach_area.shutdown();
    #endif
}

void BestPoseFinder::startExp(novelti::StartExperiment::Request& req) {
    resolution = req.map.info.resolution;
    max_dist = (int)floor(max_dist_float/resolution);

    pose_best = geometry_msgs::PoseStamped();
    pose_best.header.frame_id = "/map";
    
    reach_area.header.frame_id = "/map";
    reach_area.info.width  = 2*(max_dist+1)+1;
    reach_area.info.height = reach_area.info.width;
    reach_area.info.resolution = resolution;
    reach_area.data = vector<float>(reach_area.info.width*reach_area.info.height, REACH_AREA_UNREACHABLE);
    

    
    new (&cmap) CompoundMap(req.map.info.width, req.map.info.height);
    for (int x=0; x<req.map.info.width; x++)
        for (int y=0; y<req.map.info.height; y++)
            if (req.map.data[x + y*req.map.info.width]==0)
                cmap.setPixel(x,y, FREED); //free
    //updateVertex(req.init_pose, cur_vertex.x, cur_vertex.y);
    pose_current_lock_.lock();
        pose_current = req.init_pose;
    pose_current_lock_.unlock();
    Point cur_vertex;
    if (!getCurVertex(cur_vertex.x, cur_vertex.y))
        ROS_ERROR("%s: failed to find an accessible vertex for the initial pose", getName().c_str());
    r2a = Point(cur_vertex.x-max_dist-1, cur_vertex.y-max_dist-1);
    
    pub_pose_best = node.advertise<geometry_msgs::PoseStamped>("/pose_best", 1, true); //not latched
    sub_pose_cur  = node.subscribe("/pose_current", 1, &BestPoseFinder::poseCurCallback, this);
    if (isNode) {
        sub_pdf       = node.subscribe("/pdf", 1, &BestPoseFinder::pdfCallback, this);
    }
    #ifdef DEBUG_POSE_FINDER
        pub_reach_area = node.advertise<novelti::FloatMap>("/debug_reach_area", 1, false); //not latched
        pub_pose_debug = node.advertise<geometry_msgs::PoseStamped>("/debug_pose", 1, false); //not latched
    #endif
}

void BestPoseFinder::poseCurCallback(geometry_msgs::PoseStamped pose) {
    //ROS_INFO("%s: received /pose_current", getName().c_str());
    pose_current_lock_.lock();
        pose_current = pose.pose;
    pose_current_lock_.unlock();
//     cur_vertex_lock_.lock();
//         updateVertex(pose->pose, cur_vertex.x, cur_vertex.y);
//     cur_vertex_lock_.unlock();
}

void BestPoseFinder::pdfCallback(novelti::FloatMapConstPtr pdf){
    ROS_INFO("%s: received pdf, starting to look for best pose", getName().c_str());
    if (!calcReachArea()) //if we failed to calculate reachability area
        return;
    //ROS_INFO("%s: starting to look for the best pose", getName().c_str());
    findBestPose(pdf); //outputs to pt wrt reach_area
    pt.x+=r2a.x; pt.y+=r2a.y;
    SynchronizableNode::updatePose(pose_best, pt.x, pt.y, resolution);
    pub_pose_best.publish(pose_best);
    ros::spinOnce();
    ROS_INFO("%s: found best vertex=(%d,%d), published pose=(%f,%f)", getName().c_str(), pt.x, pt.y, pose_best.pose.position.x, pose_best.pose.position.y);
    pose_best.header.seq++;
}

bool BestPoseFinder::getCurVertex(int& cx, int& cy) {
    /* it may happen (and does happen sometimes) that due to math rounding and localization error,
     * the pose real values (x,y) would fall on an inaccessible vertex on the grid. 
     * In this case, we just need to find the nearest accessible vertex.*/
    double px, py;
    pose_current_lock_.lock();
        SynchronizableNode::updateVertex(pose_current, cx, cy, resolution);
        px = pose_current.position.x;
        py = pose_current.position.y;
    pose_current_lock_.unlock();
    double dx, dy, d, dmin = std::numeric_limits<double>::max();
    if (cmap.isSurroundedByObstacles(cx,cy)) {
        ROS_WARN("SEARCH for accessible vertex center=(%d,%d)", cx,cy);
        for (int x=max(0,cx-pose_to_vertex_tolerance); x<=min(cmap.width()-1, cx+pose_to_vertex_tolerance); x++) {
            for (int y=max(0,cy-pose_to_vertex_tolerance); y<=min(cmap.height()-1, cy+pose_to_vertex_tolerance); y++) {
                ROS_WARN("try: (%d,%d)", x,y);
                if (!cmap.isSurroundedByObstacles(x,y)) {
                    dx = px - resolution*x;
                    dy = py - resolution*y;
                    d = dx*dx + dy*dy;
                    ROS_WARN("accessible! d=%f, dmin=%f", d, dmin);
                    if (d<dmin) {
                        dmin = d;
                        cx = x;
                        cy = y;
                    }
                }
            }
        }
        if (dmin == std::numeric_limits<double>::max()){ 
            ROS_WARN("%s: could not find an accessible vertex nearby current pose (x,y)=(%f,%f), checked %d vertices in all directions", getName().c_str(), px, py, pose_to_vertex_tolerance);
            return false;
        }
    }
    return true;
}

bool BestPoseFinder::calcReachArea() {
    Point center;
    if (!getCurVertex(center.x, center.y))
        return false;
    r2a = Point(center.x-max_dist-1, center.y-max_dist-1);
    //ROS_INFO("%s: --------------------- center =(%d,%d)", getName().c_str(), center.x, center.y);
    CWave2 cw(cmap);
    CWave2Processor dummy;
    cw.setProcessor(&dummy);
    cw.calc(center, 2*max_dist);
    ra_min = Point(max(1, -r2a.x), max(1, -r2a.y) );
    int ra_size = 2*(max_dist+1);
    ra_max = Point( min(ra_size, cmap.width()-r2a.x),  min(ra_size, cmap.height()-r2a.y) );
    n_unassigned = 0;
    
    for (int x=ra_min.x; x<ra_max.x; x++)
        for (int y=ra_min.y; y<ra_max.y; y++)
            if (cmap.getPoint(x+r2a.x, y+r2a.y) != MAP_POINT_UNEXPLORED) {
                reach_area.data[x + y*reach_area.info.width] = REACH_AREA_UNASSIGNED;
                n_unassigned++;
            } else {
                reach_area.data[x + y*reach_area.info.width] = REACH_AREA_UNREACHABLE;
            }
    cmap.clearDist();
    for (int x=1; x<ra_min.x; x++)
        for (int y=1; y<ra_size; y++)
            reach_area.data[x + y*reach_area.info.width] = REACH_AREA_UNREACHABLE;
    for (int x=ra_max.x; x<ra_size; x++)
        for (int y=1; y<ra_size; y++)
            reach_area.data[x + y*reach_area.info.width] = REACH_AREA_UNREACHABLE;
    for (int y=1; y<ra_min.y; y++)
        for (int x=1; x<ra_size; x++)
            reach_area.data[x + y*reach_area.info.width] = REACH_AREA_UNREACHABLE;
    for (int y=ra_max.y; y<ra_size; y++)
        for (int x=1; x<ra_size; x++)
            reach_area.data[x + y*reach_area.info.width] = REACH_AREA_UNREACHABLE;
    
    reach_area.info.origin.position.x = resolution*(r2a.x-0.5);
    reach_area.info.origin.position.y = resolution*(r2a.y-0.5);
    #ifdef DEBUG_POSE_FINDER
        pub_reach_area.publish(reach_area);
        ros::spinOnce();
    #endif
    return true;
}

void BestPoseFinder::moveToClosestInReachAreaEuc() {
    //input (pt)  wrt to map
    //output (pt) wrt to reach_area
    Point out;
    double d, dmin = std::numeric_limits<double>::max();
    int x2, y2;
    for (int x=ra_min.x; x<ra_max.x; x++) {
        for (int y=ra_min.y; y<ra_max.y; y++) {
            if (reach_area.data[x+y*reach_area.info.width] != REACH_AREA_UNREACHABLE) {
                x2 = x+r2a.x-pt.x;
                y2 = y+r2a.y-pt.y;
                d = sqrt(x2*x2+y2*y2);
                if (d < dmin) {
                    out.x=x; out.y=y;
                    dmin = d;
                }
            }
        }
    }
    pt = out;
}

void BestPoseFinder::moveToClosestInReachAreaObst() {
    //input (pt)  wrt to map
    //output (pt) wrt to reach_area    
    CWave2 cw(cmap);
    CWave2Processor dummy;
    cw.setProcessor(&dummy);
    cw.calc(pt);
    
    Point out;
    int d, dmin = std::numeric_limits<int>::max();
    int x2, y2;
    for (int x=ra_min.x; x<ra_max.x; x++) {
        for (int y=ra_min.y; y<ra_max.y; y++) {
            if (reach_area.data[x+y*reach_area.info.width] != REACH_AREA_UNREACHABLE) {
                d = cmap.getPoint(x+r2a.x, y+r2a.y);
                //ROS_INFO("d(%d,%d)=%d, dmin=%d",x,y,d,dmin);
                if (d < dmin) {
                    out.x=x; out.y=y;
                    dmin = d;
                    //ROS_INFO("              UPDATED dmin = %d",dmin);
                }
            }
        }
    }
    cmap.clearDist();
    pt = out;
}

void BestPoseFinder::moveToClosestOnMap(novelti::FloatMapConstPtr pdf) {
    //input (pt)  wrt to map
    //output (pt) wrt to reach_area    
    Point out;
    double d, dmin = std::numeric_limits<double>::max();
    for (int x=0; x<pdf->info.width; x++) {
        for (int y=0; y<pdf->info.height; y++) {
            if (pdf->data[x+y*pdf->info.width] >= 0.0) {
                d = sqrt((x-pt.x)*(x-pt.x)+(y-pt.y)*(y-pt.y));
                if (d < dmin) {
                    out.x=x; out.y=y;
                    dmin = d;
                }
            }
        }
    }
    pt = out;
}

// void BestPoseFinder::makeWrtMap() {
//     
// }
// 
// void BestPoseFinder::makeWrtReachArea() {
// }


void BestPoseFinder::findBestPose(novelti::FloatMapConstPtr pdf1) { //no move
    if (!getCurVertex(pt.x, pt.y))
        return;
    pt.x -= r2a.x;
    pt.y -= r2a.y;
}

#ifdef DEBUG_POSE_FINDER
    void BestPoseFinder::pubDebugPose(int x, int y, bool wrtMap) {
        if (!wrtMap) {
            x += r2a.x; y += r2a.y;
        }
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "/map";
        SynchronizableNode::updatePose(pose, x, y, resolution);
        pub_pose_debug.publish(pose);
        ros::spinOnce();
        ROS_INFO("%s: published /debug_pose best vertex wrt to map: (%d,%d), published pose=(%f,%f)", getName().c_str(), x, y, pose.pose.position.x, pose.pose.position.y);
    }
#endif