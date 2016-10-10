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
    //const int OCT2POINT[8][2] = {{1,1}, {0,1}, {-1,1}, {-1,0}, {-1,-1},  {0,-1}, {1,-1},  {1,0}};
    const int OCT2POINT[8][2] = {{1,1}, {0,1}, {-1,1}, {-1,0}, {-1,-1}, {0,-1}, {1,-1},  {1,0}};
    const int OCT2POINT_MOVE[8][2] = {{0,1}, {55,55}, {-1,0}, {55,55}, {0,-1}, {55,55}, {1,0},  {55,55}};
    
    CompoundMap cmap;
    int region;
    double prob;
    
    Point wp; //walker point/position
    char woct; //walker orientation (octant: 0, 1...7)
    WalkerState wstate; //walker state
    int wstar;
    
    vector<int> branch1;
    vector<int> branch2;
    
    #ifdef EXTREMAL_MAP_DIVIDER_DEBUG
        IntMap track_map_msg;
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
        #ifdef EXTREMAL_MAP_DIVIDER_DEBUG
            track_map_msg = map_divided; //copy
        #endif
        //track_stars = std::vector<MyStar>(50);
    }
    

    
    void divide() {
        //beforeCWave();
        map_divided.data = std::vector<int>(map_divided.info.width*map_divided.info.height, 255);
        CWave2 cw(cmap);
        cw.setProcessor(this);
        Point center(vx.x,vx.y);
        cw.calc(center);
        //afterCwave();
        #ifdef EXTREMAL_MAP_DIVIDER_DEBUG
            track_map_msg.data = cmap.track_map;
            pub_debug_track_map.publish(track_map_msg);
        #endif
        prob = 0.0;
        region = 0;
        bool moved=true;
        Point start = boundaryWalkerInit(center);
        do {
            if (moved)
                processBoundaryVertex();
            #ifdef EXTREMAL_MAP_DIVIDER_DEBUG
                publishBoundaryPose();
                pub_map_div.publish(map_divided);
            #endif
            moved = boundaryWalkerUpdate();
        } while (!boundaryWalkerLooped(start));
        probs_actual[region] = prob;
        
        cmap.clearDist();
        cmap.clearTrack();
        
        float p0=probs_actual[0], p1=probs_actual[1], p2=probs_actual[2], p3=probs_actual[3];
        ROS_INFO("%s: probs_actual=[%f,%f,%f,%f], sum=%f", getName().c_str(), p0,p1,p2,p3, p0+p1+p2+p3);
    }
    
    #define LINE_WALK_MACRO(xx, yy) \
        do {\
            if (leps>=0) {leps -= dx; y++;}\
            leps += dy; x++;\
            visitVertex(s.x+(xx), s.y+(yy));\
        } while (x<dx); \
        break;
    
    void processBoundaryVertex() {
        TrackStar s = cmap.getTrackStar(cmap.getTrackStarId(wp.x, wp.y));
        int dx = wp.x-s.x, 
            dy = wp.y-s.y,
            x, y;
        char oct;
        
        if (abs(dx)>=abs(dy))
            if (dx>0)
                if (dy>=0)  { x=+dx; y=+dy; oct=0+(dx==dy);}
                else        { x=+dx; y=-dy; oct=7;}
            else
                if (dy>0)   { x=-dx; y=+dy; oct=3;}
                else        { x=-dx; y=-dy; oct=4+(dx==dy);}
        else
            if (dy>=0)
                if (dx>0)   { x=+dy; y=+dx; oct=1;}
                else        { x=+dy; y=-dx; oct=2;}
            else
                if (dx>=0)  { x=-dy; y=+dx; oct=6;}
                else        { x=-dy; y=-dx; oct=5;}
    
        dx=x; dy=y; x=0; y=0;
        int leps = dy - (oct%2==0 ? dx : 1);
        
        visitVertex(s.x, s.y);
        switch(oct) {
            case 0: LINE_WALK_MACRO( x,  y); 
            case 1: LINE_WALK_MACRO( y,  x); 
            case 2: LINE_WALK_MACRO(-y,  x); 
            case 3: LINE_WALK_MACRO(-x,  y); 
            case 4: LINE_WALK_MACRO(-x, -y); 
            case 5: LINE_WALK_MACRO(-y, -x); 
            case 6: LINE_WALK_MACRO( y, -x); 
            case 7: LINE_WALK_MACRO( x, -y); 
        }/**/
    }
    
    
    void visitVertex(int x, int y) {
        double p;
        int cur = map_divided.data[x + y*map_divided.info.width];
        if (cur == 255) {
            p = pdf->data[x + y*pdf->info.width];
            prob += p;
            map_divided.data[x + y*map_divided.info.width] = region;
            if ( prob >= probs_optimal[region] ) {
                probs_actual[region] = prob;
                prob = 0.0;
                region++;
            }
        }
    }
    
    int findLowestCommonAncestor(int s1, int s2) {
        branch1.resize(0);
        branch2.resize(0);
        int s = s1;
        while (true) {
            branch1.push_back(s);
            if (s==0) 
                break;
            s = getParentStar(s);
        }
        s = s2;
        while (true) {
            branch2.push_back(s);
            if (s==0) 
                break;
            s = getParentStar(s);
        }
        int k1=branch1.size()-1, k2=branch2.size()-1;
        while (branch1[k1] == branch2[k2]) {
            k1--; k2--;
        }
        s = branch1[k1+1];
        return s;
    }
    
    bool isContinuousTurnPath(vector<int>& path, Point& start, int end_id, bool isClockwise) {
        int k=0;
        TrackStar s2, s1 = cmap.getTrackStar(path[k]);
        int v1x = start.x-s1.x, v1y = start.y-s1.y;
        int v2x, v2y;
        int crit;
        while (path[k] != end_id) {
            k++;
            s2 = cmap.getTrackStar(path[k]);
            v2x = s1.x-s2.x;    v2y = s1.y-s2.y;
            crit = v1x*v2y - v1y*v2x;
            if (isClockwise)
                crit = -crit;
            if (crit<0)
                return false;
            v1x = v2x;  v1y = v2y;
            s1 = s2;
        }
        return true;
    }
            
    bool isStarBetweenVectors(Point& rightVec, Point& leftVec, TrackStar& pivot, TrackStar star) {
        Point testVec(star.x-pivot.x, star.y-pivot.y);
        return (rightVec.x*testVec.y - rightVec.y*testVec.x>=0) &&
               (testVec.x*leftVec.y - testVec.y*leftVec.x>=0);
    }
    
    bool allWayPointsBetweenVectors(Point& rightVec, Point& leftVec, TrackStar& pivot, vector<int>& waypoints, int end_id) {
        int k=0;
        while(waypoints[k]!=end_id) {
            if (!isStarBetweenVectors(rightVec, leftVec, pivot, cmap.getTrackStar(waypoints[k])))
                return false;
            k++;
        }
        return true;
    }
    
    int getParentStar(int star_id) { //returns id of the star that is the parent of the star with id==star_id
        TrackStar s = cmap.getTrackStar(star_id);
        return cmap.getTrackStarId(s.x, s.y);
    }
    
    bool isPointIn(Point& pt) {
        int s = cmap.getTrackStarId(pt.x, pt.y);
        if (s==wstar) {
            return true;
        } else if (s!=0 && getParentStar(s)==wstar) {//parent->child
            TrackStar child  = cmap.getTrackStar(s);
            TrackStar parent = cmap.getTrackStar(wstar);
            int Qx=wp.x-parent.x,       Qy=wp.y-parent.y;
            int Rx=pt.x-parent.x,       Ry=pt.y-parent.y;
            int Cx=child.x-parent.x,    Cy=child.y-parent.y;
/*            int cx = pt.x-child.x,      cy = pt.y-child.y;
            int px = child.x-parent.x,  py = child.y-parent.y;
            if ((child.x==wp.x && child.y==wp.y)||px*cy-py*cx >= 0) {*/
            if ((Qx*Cy-Qy*Cx)*(Cx*Ry-Cy*Rx)>=0) {
                wstar = s;
                return true;
            }
        } else if (wstar!=0 && getParentStar(wstar)==s) {//child->parent
            TrackStar child  = cmap.getTrackStar(wstar);
            TrackStar parent = cmap.getTrackStar(s);
            /*int cx = wp.x-child.x,      cy = wp.y-child.y;
            int px = child.x-parent.x,  py = child.y-parent.y;
            if ((child.x==pt.x && child.y==pt.y  ) || cx*py-cy*px >= 0) {*/
            int Qx=wp.x-parent.x,       Qy=wp.y-parent.y;
            int Rx=pt.x-parent.x,       Ry=pt.y-parent.y;
            int Cx=child.x-parent.x,    Cy=child.y-parent.y;
            if ((Qx*Cy-Qy*Cx)*(Cx*Ry-Cy*Rx)>=0) {
                wstar = s;
                return true;
            }
        } else {
            int lca = findLowestCommonAncestor(wstar, s);
            TrackStar pivotStar = cmap.getTrackStar(lca);
            Point rightVec(wp.x-pivotStar.x, wp.y-pivotStar.y);
            Point leftVec(pt.x-pivotStar.x, pt.y-pivotStar.y);
            if (allWayPointsBetweenVectors(rightVec, leftVec, pivotStar, branch1, lca) &&
                allWayPointsBetweenVectors(rightVec, leftVec, pivotStar, branch2, lca)) {
            /*if (isContinuousTurnPath(branch1, wp, lca, false) &&
                isContinuousTurnPath(branch2, pt, lca, true)) {*/
                wstar = s;
                return true;
            }
        }
        return false;
    }
    
    Point findBoundaryVertex(Point pt) {
        while (cmap.getTrackStarId(pt.x+1, pt.y) == 0) {
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
                wstate = StraightReach;
                pt.x = wp.x+OCT2POINT[woct][0];
                pt.y = wp.y+OCT2POINT[woct][1];
                if (isPointIn(pt)) {
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
                        wp.x += OCT2POINT_MOVE[woct][0];
                        wp.y += OCT2POINT_MOVE[woct][1];
                        woct -= 1; woct &= 7; //turn by -45deg
                        moved = true;
                    } else {
                        woct += 1; woct &= 7; //turn by +45deg
                    }
                }
                return moved;
        }
    }
    
    
    #ifdef EXTREMAL_MAP_DIVIDER_DEBUG
        void publishBoundaryPose() {
            //ROS_WARN("Boundary pose: (%d,%d), oct=%d", wp.x, wp.y, woct);
            geometry_msgs::PoseStamped msg = Vertex::toPose(wp.x, wp.y, map_divided.info.resolution);
            msg.header.frame_id="/map";
            msg.pose.orientation = tf::createQuaternionMsgFromYaw((double)(woct)*M_PI/4);
            pub_debug_pose_border.publish(msg);
        }
    #endif
    
};



}
