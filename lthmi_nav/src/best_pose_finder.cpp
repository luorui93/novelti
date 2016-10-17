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

#include <lthmi_nav/best_pose_finder.h>
#include <limits>


using namespace lthmi_nav;

BestPoseFinder::BestPoseFinder() :
        SynchronizableNode()
{
    double max_vel, period, safety_coef;
    node.param<double>("max_vel", max_vel, 0.0);
    node.param<double>("safety_coef", safety_coef, 0.0);
    node.param<double>("period", period, 0.0);
    max_dist_float = max_vel*safety_coef*period;
    if (max_dist_float==0.0)
        throw ros::Exception("ERROR: At least one of the following node parameters (max_vel, period, safety_coef) is not specified or 0.0. All must be greater than zero.");
}
    
void BestPoseFinder::stop() {
    pub_pose_best.shutdown();
    sub_pose_cur.shutdown();
    sub_pdf.shutdown();
    #ifdef DEBUG_POSE_FINDER
        pub_reach_area.shutdown();
    #endif
}

void BestPoseFinder::start(lthmi_nav::StartExperiment::Request& req) {
    resolution = req.map.info.resolution;
    max_dist = (int)floor(max_dist_float/resolution);

    reach_area.header.frame_id = "/map";
    reach_area.info.width  = 2*(max_dist+1)+1;
    reach_area.info.height = reach_area.info.width;
    reach_area.info.resolution = resolution;
    reach_area.data = vector<float>(reach_area.info.width*reach_area.info.height, REACH_AREA_UNREACHABLE);
    
    new (&cur_vertex) Vertex(req.init_pose, resolution);
    r2a = Point(cur_vertex.x-max_dist-1, cur_vertex.y-max_dist-1);
    new (&cmap) CompoundMap(req.map.info.width, req.map.info.height);
    for (int x=0; x<req.map.info.width; x++)
        for (int y=0; y<req.map.info.height; y++)
            if (req.map.data[x + y*req.map.info.width]==0)
                cmap.setPixel(x,y, FREED); //free
    
    pub_pose_best = node.advertise<geometry_msgs::PoseStamped>("/pose_best", 1, false); //not latched
    sub_pose_cur  = node.subscribe("/pose_current", 1, &BestPoseFinder::poseCurCallback, this);
    sub_pdf       = node.subscribe("/pdf", 1, &BestPoseFinder::pdfCallback, this);
    #ifdef DEBUG_POSE_FINDER
        pub_reach_area   = node.advertise<lthmi_nav::FloatMap>("/debug_reach_area", 1, false); //not latched
    #endif
}

void BestPoseFinder::poseCurCallback(const geometry_msgs::PoseStamped& pose) {
    ROS_INFO("%s: received pose", getName().c_str());
    new (&cur_vertex) Vertex(pose.pose, resolution);
    r2a = Point(cur_vertex.x-max_dist-1, cur_vertex.y-max_dist-1);
}

void BestPoseFinder::pdfCallback(lthmi_nav::FloatMapConstPtr pdf){
    ROS_INFO("%s: received pdf", getName().c_str());
    calcReachArea();
    ROS_INFO("%s: starting to look for the best pose", getName().c_str());
    findBestPose(pdf); //outputs to pt wrt reach_area
    ROS_INFO("=-------------------found best vertex=(%d,%d), published", pt.x, pt.y);
    geometry_msgs::PoseStamped pose = Vertex::toPose(pt.x+r2a.x, pt.y+r2a.y, resolution);
    pose.header.frame_id = "/map";
    pub_pose_best.publish(pose);
    ROS_INFO("%s: found best vertex=(%d,%d), published pose=(%f,%f)", getName().c_str(), pt.x, pt.y, pose.pose.position.x, pose.pose.position.y);
}

void BestPoseFinder::calcReachArea() {
    Point center(cur_vertex.x, cur_vertex.y);
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
    #endif
}

void BestPoseFinder::moveToClosestInReachAreaEuq() {
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
                if (d < dmin) {
                    out.x=x; out.y=y;
                    dmin = d;
                }
            }
        }
    }
    cmap.clearDist();
    pt = out;
}

void BestPoseFinder::moveToClosestOnMap(lthmi_nav::FloatMapConstPtr pdf) {
    //input (pt)  wrt to map
    //output (pt) wrt to reach_area    
    Point out;
    double d, dmin = std::numeric_limits<double>::max();
    for (int x=0; x<pdf->info.width; x++) {
        for (int y=0; y<pdf->info.height; y++) {
            if (pdf->data[x+y*pdf->info.width] != REACH_AREA_UNREACHABLE) {
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
