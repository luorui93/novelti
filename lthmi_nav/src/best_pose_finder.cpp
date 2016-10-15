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
        pub_reach_area   = node.advertise<lthmi_nav::IntMap>("/debug_reach_area", 1, false); //not latched
    #endif
}

void BestPoseFinder::poseCurCallback(const geometry_msgs::PoseStamped& pose) {
    ROS_INFO("%s: received pose", getName().c_str());
    //vx(pose, 0.1);//(double)(pdf->info.resolution));
    new (&cur_vertex) Vertex(pose.pose, (double)(map_divided.info.resolution));
    r2a = Point(cur_vertex.x-max_dist-1, cur_vertex.y-max_dist-1);
}

void BestPoseFinder::pdfCallback(lthmi_nav::FloatMapConstPtr pdf){
    ROS_INFO("%s: received pdf", getName().c_str());
    calcReachArea();
    ROS_INFO("%s: starting to look for the best pose", getName().c_str());
    Point pt = findBestPose(pdf);
    geometry_msgs::PoseStamped pose = Vertex::toPose(pt.x+r2a.x, pt.y+r2a.y, resolution);
    pub_pose_best.publish(pose);
    ROS_INFO("%s: found best vertex=(%d,%d), published pose=(%f,%f)", getName().c_str(), pt.x, pt.y, pose.pose.position.x, pose.pose.position.y);
}

void BestPoseFinder::calcReachArea() {
    Point center(cur_vertex.x, cur_vertex.y);
    CWave2 cw(cmap);
    cw.calc(center, max_dist);
    ra_min = Point(max(1, -r2a.x), max(1, -r2a.y) );
    int ra_size = 2*(max_dist+1);
    ra_max = Point( min(ra_size, cmap.width()-r2a.x),  min(ra_size, cmap.height()-r2a.y) );
    
    for (int x=ra_min.x; x<ra_max.x; x++)
        for (int y=ra_min.y; y<ra_max.y; y++)
            if (cmap.getPoint(x+r2a.x, y+r2a.y) != MAP_POINT_UNEXPLORED)
                reach_area.data[x + y*reach_area.info.width] = REACH_AREA_UNASSIGNED;
            
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
        
    #ifdef DEBUG_POSE_FINDER
        pub_reach_area.publish(reach_area);
    #endif
}

/*#include "ros/ros.h" //for debug only
#define MEAN_DIST 1
#define PDF_STAT 1
#include "fast_dist.cpp"

#include <lthmi_nav/best_pose_finder.h>
#include <lthmi_nav/map.h>
    //#include <ncurses.h> 


namespace lthmi_nav {

    Point2D find_best_pose(
        MapIf<int>& map, 
        MapIf<float>& pdf, 
        Point2D& cur_pose, 
        int max_dist
    ) {
        int neibhours[8][2] = {{1,0}, {1,1}, {0,1}, {-1,1}, {-1,0}, {-1,-1}, {0,-1}, {1,-1}};
        ///display_float_map(pdf, 2*map.height(), map.width());
        
        ReachAreaInfo rai={0};
        MapRos<lthmi_nav::FloatMap,float> reach_area = calculate_reachability_area(map, cur_pose, max_dist, pdf, rai);
        ///mvaddch(map.height()-1-cur_pose.y, cur_pose.x, '*');  refresh(); //main area
        //display_float_map(reach_area, 0, map.width()+5);

        Point2D rp = {cur_pose.x-rai.reach_to_main.x, cur_pose.y-rai.reach_to_main.y};
        float cur_mean_dist = calculate_mean_distance(map, pdf, cur_pose, rai);
        int x,y;
        float v;
        int best_neibhour = 0;
        ///mvprintw(map.height()+1, 40, "avg cost in current robot pose (%d, %d) is %f ", cur_pose.x, cur_pose.y, cur_mean_dist);
       
        while (true) {
            ///mvprintw(map.height()+2, 0, "current pivot point: on reach area (%d,%d), on map area (%d,%d), cur_mean_dist=%f ", 
            ///            rp.x, rp.y, rp.x+rai.reach_to_main.x, rp.y+rai.reach_to_main.y,  cur_mean_dist); refresh();
            ///mvaddch(rai.reach_visual.y-rp.y, rai.reach_visual.x+rp.x, 'X'); refresh();
            best_neibhour = -1;
            for (int neib=0; neib<8; neib++) {
                x=rp.x+neibhours[neib][0];
                y=rp.y+neibhours[neib][1];
                ///mvprintw(map.height()+3, 0, "check point (%d,%d) on reach area, i.e. (%d,%d) on the map ", x, y, x+rai.reach_to_main.x, y+rai.reach_to_main.y); refresh();
                if (reach_area.get(x,y)==REACHABLE_CELL) {
                    ///mvprintw(map.height()+4, 0, "    reachable, caluclating v:");
                    v = calculate_mean_distance(map, pdf, {x+rai.reach_to_main.x, y+rai.reach_to_main.y}, rai);
                    reach_area.set(x,y,  v);
                        ///attron(COLOR_PAIR(PIXEL_COLOR_REACH_VISITED));
                        ///mvaddch(rai.reach_visual.y-y, rai.reach_visual.x+x, '.');
                        ///attroff(COLOR_PAIR(PIXEL_COLOR_REACH_VISITED));
                        ///refresh();
                    if ( v < cur_mean_dist) {
                        cur_mean_dist=v;
                        best_neibhour=neib;
                    }
                } else {
                    ///mvprintw(map.height()+4, 0, "    unreachable.");
                }
                //display_float_map(reach_area, 0, map.width()+5);
                ///mvprintw(0, 0, "");
                //getch();
            }
            if (best_neibhour>=0) {
                rp.x+=neibhours[best_neibhour][0];
                rp.y+=neibhours[best_neibhour][1];

            }
            else break;
        } 
        
        /*mvaddch(map.height()-1-cur_pose.y, cur_pose.x, 'S');
        mvaddch(map.height()-1-(rp.y+rai.reach_to_main.y), rp.x+rai.reach_to_main.x, 'E');
        mvprintw(map.height()+9, 0, "Avg cost at end point (%d,%d) is %f", rp.x+rai.reach_to_main.x, rp.y+rai.reach_to_main.y, cur_mean_dist);
        refresh();* /
        return {rp.x+rai.reach_to_main.x, rp.y+rai.reach_to_main.y};
    }
    
    MapRos<lthmi_nav::FloatMap,float> calculate_reachability_area(MapIf<int>& map, Point2D& cur_pose, int max_dist, MapIf<float>& pdf, ReachAreaInfo& rai) {
            calculate_distances(map, cur_pose.x, cur_pose.y, 2*max_dist, pdf); //TODO remove pdf
                  // reach_to_main;                               reach_size;                      reach_visual; 
            rai = { {cur_pose.x-max_dist, cur_pose.y-max_dist},   {2*max_dist+1, 2*max_dist+1},   {map.width()+5, 2*max_dist+1} }; 
            
            int x_min = max(0, rai.reach_to_main.x);
            int y_min = max(0, rai.reach_to_main.y);
            int x_max = min(map.width()-1, cur_pose.x+max_dist);
            int y_max = min(map.height()-1, cur_pose.y+max_dist);
            
            MapRos<lthmi_nav::FloatMap,float> reach_area(rai.reach_size.x, rai.reach_size.y, UNREACHABLE_CELL);
            
            PixelColorPair color;

            for (int x=x_min; x<=x_max; x++) {
                for (int y=y_min; y<=y_max; y++) {
                    if (map.get(x,y)>=0) {
                        map.set(x,y,   MAP_CELL_EMPTY);
                        reach_area.set(x-rai.reach_to_main.x,y-rai.reach_to_main.y,   REACHABLE_CELL);
                        //color=PIXEL_COLOR_REACH_NOT_VISITED;
                    } else {
                        //color=PIXEL_COLOR_UNREACH;
                    }
                    /*attron(COLOR_PAIR(color));
                    mvaddch(rai.reach_visual.y-(y-rai.reach_to_main.y), rai.reach_visual.x+(x-rai.reach_to_main.x), '.'); //reach area
                    mvaddch(map.height()-1-y, x, '.'); //main area
                    attroff(COLOR_PAIR(color));* /
                }
            }
            //refresh();
            return reach_area;
    }
    
    float calculate_mean_distance(MapIf<int>& map, MapIf<float>& pdf, Point2D pose_candidate, ReachAreaInfo& rai) {
        //ROS_INFO("best_pose_finder: try pose candidate at (%d,%d)", pose_candidate.x, pose_candidate.y);
        Galaxy glx = calculate_distances(map, pose_candidate.x, pose_candidate.y, -1, pdf);
        //ROS_INFO("best_pose_finder: got pose candidate (%d,%d) -> glx.mean_dist=%f", pose_candidate.x, pose_candidate.y, glx.mean_dist);
            /*attron(COLOR_PAIR(PIXEL_COLOR_REACH_VISITED));
            //mvaddch(rai.reach_visual.y-(pose_candidate.y-rai.reach_to_main.y), rai.reach_visual.x+(pose_candidate.x-rai.reach_to_main.x), '.');
            mvaddch(map.height()-1-pose_candidate.y, pose_candidate.x, '.');
            attroff(COLOR_PAIR(PIXEL_COLOR_REACH_VISITED));
        mvprintw(map.height()+6, 0, "mean dist at reach area (%d,%d) (=map (%d,%d)) is %f", pose_candidate.x-rai.reach_to_main.x, pose_candidate.y-rai.reach_to_main.y, pose_candidate.x, pose_candidate.y, glx.mean_dist); refresh();* /
        map.clean_dist();
        return glx.pdf_stat;
    }
    
    void display_float_map(vector<vector<float>>& m, int row, int col) {
        float p;
        int w = m.size();
        int h = m[0].size();
        PixelColorPair color;
        for (int x=0;x<w;x++) {
            for (int y=0;y<h;y++) {
                p = m[x][y];
                if (p==UNREACHABLE_CELL) 
                    color = PIXEL_COLOR_UNREACH;
                else if (p==REACHABLE_CELL)
                    color = PIXEL_COLOR_REACH_NOT_VISITED;
                else
                    color = PIXEL_COLOR_REACH_VISITED;
                attron(COLOR_PAIR(color));
                mvaddch(row+h-1-y, col+x, '.');
                attroff(COLOR_PAIR(color));
            }
        }
        refresh();
    }
    
}
*/