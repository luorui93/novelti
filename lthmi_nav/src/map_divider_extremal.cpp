#define EXTREMAL_MAP_DIVIDER_DEBUG 1
        /* +--------------------------------+
         * |                 \              |
         * |  ____            \             |
         * | /    \           |             |
         * |/      \   1      |      0      |
         * |        \        /              |
         * |         \     _/               |
         * |          \   /                 |
         * |           --+------------------|
         * |             |            _     |
         * |             |___   3    | \    |
         * |     2           \       |  \   |
         * |                  \______|   \  |
         * |                              \ |
         * |                               \|
         * |                                |
         * +--------------------------------+  */
        
#include <lthmi_nav/map_divider.h>
#include <CompoundMap.h>
#include <CWave2.h>

#ifdef EXTREMAL_MAP_DIVIDER_DEBUG
    #include <tf/transform_datatypes.h>
#endif    


using namespace cwave;

namespace lthmi_nav {


class ExtremalMapDivider :  public MapDivider, public CWave2Processor {
public:
    enum WalkerState {DiagUnreach, DiagReach, StraightReach};
    typedef struct {
        int x;
        int y;
        double dist;
    } MyStar;
    
    const int OCT2CELL[8][2] = {{0,0}, {-1,0}, {-1,0}, {-1,-1}, {-1,-1},  {0,-1}, {0,-1},  {0,0}}; 
    const int OCT2POINT[8][2] = {{1,1}, {0,1}, {-1,1}, {-1,0}, {-1,-1},  {0,-1}, {1,-1},  {1,0}};
    
    CompoundMap cmap;
    std::vector<MyStar> track_stars;
    IntMap track_map;
    int region;
    double prob;
    
    Point wp; //walker point/position
    char woct; //walker orientation (octant: 0, 1...7)
    WalkerState wstate; //walker state
    int wstar;
    
    #ifdef EXTREMAL_MAP_DIVIDER_DEBUG
        ros::Publisher pub_debug_pose_border;
        ros::Publisher pub_debug_track_map;
    #endif
    
    
    ExtremalMapDivider() :
        MapDivider() 
    { 
        #ifdef EXTREMAL_MAP_DIVIDER_DEBUG
            pub_debug_pose_border   = node.advertise<geometry_msgs::PoseStamped>("/debug_pose_border", 1, false); //not latched
            pub_debug_track_map   = node.advertise<IntMap>("/debug_track_map", 1, false); //not latched
        #endif
    }
    
    void start(lthmi_nav::StartExperiment::Request& req) {
        new (&cmap) CompoundMap(req.map.info.width, req.map.info.height);
        for (int x=0; x<req.map.info.width; x++)
            for (int y=0; y<req.map.info.height; y++)
                if (req.map.data[x + y*req.map.info.width]==0)
                    cmap.setPixel(x,y, FREED); //free
        MapDivider::start(req);
        track_map = map_divided; //copy
        //track_stars = std::vector<MyStar>(50);
    }
    

    
    void divide() {
        //beforeCWave();
        CWave2 cw(cmap);
        cw.setProcessor(this);
        Point center(vx.x,vx.y);
        ROS_WARN("----------------, (%d,%d)", vx.x,vx.y);
        cw.calc(center);
        //afterCwave();
        #ifdef EXTREMAL_MAP_DIVIDER_DEBUG
            pub_debug_track_map.publish(track_map);
        #endif
        bool moved;
        Point start = boundaryWalkerInit(center);
        do {
            #ifdef EXTREMAL_MAP_DIVIDER_DEBUG
                publishBoundaryPose();
            #endif
            moved = boundaryWalkerUpdate();
        } while (!boundaryWalkerLooped(start));

        cmap.clearDist();
        //ROS_INFO("probs_actual=[%f,%f,%f,%f]", probs_actual[0], probs_actual[1], probs_actual[2], probs_actual[3]);
    }
    
    int getParentStar(int star_id) { //returns id of the star that is the parent of the star with id==star_id
        return track_map.data[ track_stars[star_id].x  +   track_stars[star_id].y*track_map.info.width];
    }
    
    bool isPointIn(Point& pt) {
        int s = track_map.data[ pt.x  +   pt.y*track_map.info.width];
        if (s==wstar) {
            return true;
        } else if (s!=0 && getParentStar(s)==wstar) {
            wstar =s;
            return true;
        } else if (wstar!=0 && getParentStar(wstar)==s) {
            wstar = s;
            return true;
        }
        return false;
    }
    
    Point findBoundaryVertex(Point pt) {
        while (track_map.data[pt.x+1 + pt.y* track_map.info.width] == 0) {
            pt.x++;
        }
        return pt;
    }
    
    Point boundaryWalkerInit(Point& center) {//returns first walker
        wp = findBoundaryVertex(center);
        woct = 1;
        wstate = DiagUnreach;
        wstar = 0;
        return wp;
    }
    
    bool boundaryWalkerLooped(Point& start) {
        return start.x==wp.x && start.y==wp.y && woct==1;
    }
    
    bool boundaryWalkerUpdate() {
        bool moved = false;
        Point pt;
        switch (wstate) {
            case DiagUnreach:
                if (cmap.isPixelOccupied(wp.x+OCT2CELL[woct][0], wp.y+OCT2CELL[woct][1])) {
                    woct += 2; woct &= 7; //turn by +90deg
                } else {
                    pt.x = wp.x+OCT2POINT[woct][0];
                    pt.y = wp.y+OCT2POINT[woct][1];
                    if (isPointIn(pt)) {
                        woct -= 2; woct &= 7; //turn by -90deg
                        wp = pt;
                        moved = true;
                    } else {
                        wstate = StraightReach;
                        woct += 1; woct &= 7; //turn by +45deg
                    }
                }
                return moved;
            case DiagReach:
                pt.x = wp.x+OCT2POINT[woct][0];
                pt.y = wp.y+OCT2POINT[woct][1];
                if (isPointIn(pt)) {
                    wstate = StraightReach;
                    woct -= 1; woct &= 7; //turn by -45deg
                    wp = pt;
                    moved = true;
                } else {
                    woct += 1; woct &= 7; //turn by +45deg
                }
                return moved;
            case StraightReach:
                if (cmap.isPixelOccupied(wp.x+OCT2CELL[woct][0], wp.y+OCT2CELL[woct][1])) {
                    woct += 1; woct &= 7; //turn by +45deg
                    wstate = DiagUnreach;
                } else {
                    wstate = DiagReach;
                    pt.x = wp.x+OCT2POINT[woct][0];
                    pt.y = wp.y+OCT2POINT[woct][1];
                    if (isPointIn(pt)) {
                        woct += 1; woct &= 7; //turn by +45deg
                    } else {
                        woct -= 1; woct &= 7; //turn by -45deg
                        moved = true;
                    }
                }
                return moved;
        }
    }
    
    
    #ifdef EXTREMAL_MAP_DIVIDER_DEBUG
        void publishBoundaryPose() {
            ROS_WARN("Boundary pose: (%d,%d), oct=%d", wp.x, wp.y, woct);
            geometry_msgs::PoseStamped msg = Vertex::toPose(wp.x, wp.y, map_divided.info.resolution);
            msg.header.frame_id="/map";
            msg.pose.orientation = tf::createQuaternionMsgFromYaw((double)(woct)*M_PI/4);
            pub_debug_pose_border.publish(msg);
        }
    #endif
    
    
    
    
    void onInitSource(Point& pt) {
        track_map.data[pt.x + pt.y*track_map.info.width] = 0;
    }

    void onDistanceCorrection(Star& star, Point& pt, int old_dist, int new_dist) {
        track_map.data[pt.x + pt.y*track_map.info.width] = star.id;
    };
    
    
    void onSetPointDistance(Star& star, OctPoint& op, bool is_nbp, Point& pt, int old_dist, int new_dist) {
        //ROS_WARN("!!!!!!!!!!!!!!!!!!!!!!!!!!!--345, (%d,%d) k=%d", pt.x, pt.y, pt.x + pt.y*track_map.info.width); 
        track_map.data[pt.x + pt.y*track_map.info.width] = star.id;
    }

    void onAddStar(Star& parent_star, Star& s) {
        track_stars.push_back({s.c.x, s.c.y, s.dist});
    }

};
}
