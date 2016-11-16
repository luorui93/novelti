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
/*
====== Issues: =======
#1 No simple boundary walk can guarantee that all vertices are visited. Proof:

        \_ C     C     C     C     C     C     C
          \_
            \_    v3     v2
              \__/      /
           A   \B\___  C     C     C     C     C
                \__  \_____
                   \____   \__
                        \_     \_
           A     A     A  \  B   \ C     C     C
                      /    \__    \______
                     v1       \_         \_______
                                \                \_______
           A     A     A     A   \ B     B     B         \______
                                  \_                            \______       Star B
                                    \                                  \____ /
                              *      \                                     *|
                             / XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX          Star C
                          Star A                                                         \
                                                                                          *
    Explanation: the continuous merge boundary forms a noose that is narrow enough to fit between 
    adjacent vertices v1 and v2 reachable from stars A and C, respectively, but still encloses
    vertex v3 reachable from star B. Thus, if the discreet boundary walker looks only at its immediate 
    neighbours, vertex v3 will missed.

#2 There are configurations when boundary walker can get stuck in an infinite loop:
    git commit 75717690a235022d8d86b86d4f4feb4b8098671d
    $ roslaunch lthmi_nav test_map_divider.launch map:=office4 debug:=1 px:=44 py:=72
    
    roslaunch lthmi_nav test_map_divider.launch map:=ak500inflated debug:=0 pdf_seed:=37 px:=46 py:=201
    roslaunch lthmi_nav test_map_divider.launch map:=ak500inflated debug:=0 pdf_seed:=454 px:=44 py:=200
    roslaunch lthmi_nav test_map_divider.launch map:=ak500inflated debug:=0 pdf_seed:=135 px:=52 py:=202
    roslaunch lthmi_nav test_map_divider.launch map:=ak500inflated debug:=0 pdf_seed:=30 px:=40 py:=200
    pdf_seed=41
41
[INFO] [WallTime: 1476133942.199209] /test_map_divider: published /pose_optimal vertex=(41,200)
*/
#include <lthmi_nav/map_divider.h>
#include <CompoundMap.h>
#include <CWave2.h>

#ifdef DEBUG_DIVIDER
    #include <tf/transform_datatypes.h>
#endif    


using namespace cwave;

namespace lthmi_nav {

/*class ExtremalMapDivCwaveProcessor : public CWave2Processor {
public:
    typedef Point BorderVertex;
    typedef std::list<BorderVertex> StarBorder;
    typedef BeamBorder std::list<>;
    
    struct BBeam {
        ...  start;
        ...  end;
    }
    
    struct SStar {
        std::list<BorderVertex>  border;
        std::list<BBeam> beams;
    }
    
    CompoundMap& prev_cmap_;
    vector<SStar> sstars_;
    
    Point cur_vertex_;
    
    ExtremalMapDivCwaveProcessor(CompoundMap& prev_cmap) :
        prev_cmap_(prev_cmap)
    {
    }
    
    void processVertex(const Point& p) {
        cur_vertex_ = p;
    }
    
    void addStartCandidate() {
        double alpha = atan2(cur_vertex_.x - cur_star_.x, cur_vertex_.y - cur_star_.y);
        if (alpha < cur_beam_.start->alpha) {
            //update
            *(cur_beam_.start) = {cur_vertex_.x, cur_vertex_.y, alpha};
        } else {
            //add
            cur_beam_.start++;//
            cur_beam_.start = cur_star_.border.insert(cur_beam_.start, {cur_vertex_.x, cur_vertex_.y, alpha});
        }
    }
    
    void addEndCandidate() {
        double alpha = atan2(cur_vertex_.x - cur_star_.x, cur_vertex_.y - cur_star_.y);
        if (alpha > cur_beam_.end->alpha) {
            //update
            *(cur_beam_.end) = {cur_vertex_.x, cur_vertex_.y, alpha};
        } else {
            //add
            cur_beam_.end++;//
            cur_beam_.end = cur_star_.border.insert(cur_beam_.end, {cur_vertex_.x, cur_vertex_.y, alpha});
        }
    }
    
    void onInitSource(Point& p) {};
    void onAddStar(Star& parent_star, Star& s) {};
    void onSetPointDistance(Star& star, OctPoint& op, bool is_nbp, Point& p, int old_dist, int new_dist) {
        if (star.id == prev_cmap_.getTrackStarId(p.x,p.y)
            processVertex(p);
    };
    void onDistanceCorrection(Star& parent_star, Point& p, int old_dist, int new_dist) {};
    void onNextStar() {
        cur_star_ = 
    };
    void onNextBeam() {};
    void onBeamDeletion() {};
    void onBeamAddition() {};
    void onBeamStart() {
        cur_beam_.
    };
    void onBeamEnd() {};
    void onRecursiveCall() {
        
    };
    void onRecursiveReturn() {}; 
}*/



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
    //int region;
    //double prob;
    
    Point wp; //walker point/position
    char woct; //walker orientation (octant: 0, 1...7)
    WalkerState wstate; //walker state
    int wstar;
    
    vector<int> branch1;
    vector<int> branch2;
    
    #ifdef DEBUG_DIVIDER
        IntMap track_map_msg;
        ros::Publisher pub_debug_pose_border;
        ros::Publisher pub_debug_track_map;
    #endif
    
    
    ExtremalMapDivider() :
        MapDivider() 
    { 
        #ifdef DEBUG_DIVIDER
            pub_debug_pose_border = node.advertise<geometry_msgs::PoseStamped>("/debug_pose_border", 1, true); //not latched
            pub_debug_track_map   = node.advertise<IntMap>("/debug_track_map", 1, true); //not latched
        #endif
    }
    
    void start(lthmi_nav::StartExperiment::Request& req) {
        new (&cmap) CompoundMap(req.map.info.width, req.map.info.height);
        for (int x=0; x<req.map.info.width; x++)
            for (int y=0; y<req.map.info.height; y++)
                if (req.map.data[x + y*req.map.info.width]==0)
                    cmap.setPixel(x,y, FREED); //free
        MapDivider::start(req);
        #ifdef DEBUG_DIVIDER
            track_map_msg = map_divided; //copy
        #endif
        //track_stars = std::vector<MyStar>(50);
    }
    
    
    void qqq(int x, int y) {};

    void reassignSeparatedVertices(CompoundMap& cmap) {
        int sid, neigh_sid, neigh_merge_vertex_sid;
        TrackStar star;
        bool separated_vertex;
        int neighs[8][2] = {{1,0},{1,1},{0,1},{-1,-1},  {-1,0},{-1,-1},{0,-1},{1,-1}};
        int neighs4[4][2] = {{1,0},{0,1},  {-1,0},{0,-1}};
        Point pt;
        Point npt;
        for (pt.x=1;pt.x<cmap.width()-1; pt.x++) {
            for (pt.y=1;pt.y<cmap.height()-1; pt.y++) {
                if (pt.x==399 && pt.y==335 || (pt.x==397 && pt.y==336))
                    qqq(pt.x,pt.y);
                sid = cmap.getTrackStarId(pt.x, pt.y);
                //ROS_DEBUG("x=%d,y=%d", pt.x, pt.y);
                if (sid>=0) {
                    star = cmap.getTrackStar(sid);
                    separated_vertex = true;
                    for (int n=0; n<4; n++) {//find neighbors 
                        npt.x = pt.x+neighs4[n][0]; npt.y = pt.y+neighs4[n][1];
                        neigh_sid = cmap.getTrackStarId(npt.x,npt.y);
                        if (neigh_sid == sid || (sid!=0 && star.x==npt.x && star.y==npt.y)) {
                            separated_vertex = false;
                            break;
                        }
                    }
                    if (separated_vertex) {
                        neigh_merge_vertex_sid = -1;
                        for (int n=0; n<4; n++) {
                            npt.x = pt.x+neighs4[n][0]; npt.y = pt.y+neighs4[n][1];
                            neigh_sid = cmap.getTrackStarId(npt.x,npt.y);
                            if (neigh_sid >=0 && !isPointIn(pt, npt)) {
                                neigh_merge_vertex_sid = neigh_sid;
                            }
                        }
                        if (neigh_merge_vertex_sid>=0) {
                            cmap.updTrackMap(pt.x,pt.y,neigh_merge_vertex_sid);
                            ROS_DEBUG("%s: fixed a separated_vertex at (%d,%d), reset star_id %d->%d", getName().c_str(), pt.x,pt.y, sid, neigh_merge_vertex_sid);
                            /*ROS_DEBUG("%s: found a separated_vertex at (%d,%d), current star_id=%d", getName().c_str(), pt.x,pt.y, sid);
                            for (int n=0; n<8; n++) {//find neighbors with assigned vertices
                                neigh_sid = cmap.getTrackStarId(pt.x+neighs[n][0], pt.y+neighs[n][1]);
                                if (neigh_sid>=0) {
                                    cmap.updTrackMap(pt.x,pt.y,neigh_sid);
                                    ROS_DEBUG("%s: fixed a separated_vertex at (%d,%d), reset star_id %d->%d", getName().c_str(), pt.x,pt.y, sid, neigh_sid);
                                    break;
                                }
                            }*/
                        }
                    }
                }
            }
        }
    }
    
    
    void divide() {
        //beforeCWave();
        //ROS_INFO_NAMED("qwert", "%s: qqqqqqqqqqqqqqqqqqqqqqq", getName().c_str());
        map_divided.data = std::vector<int>(map_divided.info.width*map_divided.info.height, 255);
        CWave2 cw(cmap);
        cw.setProcessor(this);
        cw.calc(pt_best);
        //afterCwave();
        #ifdef DEBUG_DIVIDER
            track_map_msg.data = cmap.track_map;
            pub_debug_track_map.publish(track_map_msg);
        #endif
        reassignSeparatedVertices(cmap);
        reassignSeparatedVertices(cmap);
        #ifdef DEBUG_DIVIDER
            track_map_msg.data = cmap.track_map;
            pub_debug_track_map.publish(track_map_msg);
        #endif
        //prob = 0.0;
        //region = 0;
        bool moved=true;
        Point start = boundaryWalkerInit(pt_best);
        do {
            if (moved)
                processBoundaryVertex();
            #ifdef DEBUG_DIVIDER
                publishBoundaryPose();
                pub_map_div.publish(map_divided);
            #endif
            moved = boundaryWalkerUpdate();
        } while (!boundaryWalkerLooped(start));
        //probs_actual[region] = prob;

        removeSingleUnassigned();
        cmap.clearDist();
        //cmap.clearTrack();
        
        /*double p0=probs_actual[0], p1=probs_actual[1], p2=probs_actual[2], p3=probs_actual[3];
        double sum = p0+p1+p2+p3;
        ROS_INFO("%s: probs_actual=[%f,%f,%f,%f], sum=%f", getName().c_str(), p0,p1,p2,p3, sum);
        if (fabs(sum-1.0)>0.001) {
            ROS_ERROR("%s: ERROR: sum of actual probabilities is %f which is too far from 1.0", getName().c_str(), sum);
            exit(1);
        }*/
    }
    
    #define LINE_WALK_MACRO(xx, yy) \
        while (x<dx) {\
            if (leps>=0) {leps -= dx; y++;}\
            leps += dy; x++;\
            markVertex(s.x+(xx), s.y+(yy));\
        }; \
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
        
        markVertex(s.x, s.y);
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
    
    
    /*void visitVertex(int x, int y) {
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
    }*/
    
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
    
    bool isPointIn(Point& pt, Point& wpt) {
        int s = cmap.getTrackStarId(pt.x, pt.y);
        int ws = cmap.getTrackStarId(wpt.x, wpt.y);
        if (s==ws) {
            return true;
        } else if (s!=0 && getParentStar(s)==ws) {//parent->child
            TrackStar child  = cmap.getTrackStar(s);
            TrackStar parent = cmap.getTrackStar(ws);
            int Qx=wpt.x-parent.x,       Qy=wpt.y-parent.y;
            int Rx=pt.x-parent.x,       Ry=pt.y-parent.y;
            int Cx=child.x-parent.x,    Cy=child.y-parent.y;
/*            int cx = pt.x-child.x,      cy = pt.y-child.y;
            int px = child.x-parent.x,  py = child.y-parent.y;
            if ((child.x==wpt.x && child.y==wpt.y)||px*cy-py*cx >= 0) {*/
            if ((Qx*Cy-Qy*Cx)*(Cx*Ry-Cy*Rx)>=0) {
                ws = s;
                return true;
            }
        } else if (ws!=0 && getParentStar(ws)==s) {//child->parent
            TrackStar child  = cmap.getTrackStar(ws);
            TrackStar parent = cmap.getTrackStar(s);
            /*int cx = wpt.x-child.x,      cy = wpt.y-child.y;
            int px = child.x-parent.x,  py = child.y-parent.y;
            if ((child.x==pt.x && child.y==pt.y  ) || cx*py-cy*px >= 0) {*/
            int Qx=wpt.x-parent.x,       Qy=wpt.y-parent.y;
            int Rx=pt.x-parent.x,       Ry=pt.y-parent.y;
            int Cx=child.x-parent.x,    Cy=child.y-parent.y;
            if ((Qx*Cy-Qy*Cx)*(Cx*Ry-Cy*Rx)>=0) {
                ws = s;
                return true;
            }
        } else {
            int lca = findLowestCommonAncestor(ws, s);
            TrackStar pivotStar = cmap.getTrackStar(lca);
            Point rightVec(wpt.x-pivotStar.x, wpt.y-pivotStar.y);
            Point leftVec(pt.x-pivotStar.x, pt.y-pivotStar.y);
            if (allWayPointsBetweenVectors(rightVec, leftVec, pivotStar, branch1, lca) &&
                allWayPointsBetweenVectors(rightVec, leftVec, pivotStar, branch2, lca)) {
            /*if (isContinuousTurnPath(branch1, wpt, lca, false) &&
                isContinuousTurnPath(branch2, pt, lca, true)) {*/
                ws = s;
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
                    if (isPointIn(pt,wp)) {
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
                if (isPointIn(pt,wp)) {
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
                    pt.x = wp.x+OCT2POINT[woct+1][0];
                    pt.y = wp.y+OCT2POINT[woct+1][1];
                    if (isPointIn(pt,wp)) {
                        //wp.x += OCT2POINT_MOVE[woct][0];
                        //wp.y += OCT2POINT_MOVE[woct][1];
                        wstate = DiagReach;
                        wp = pt;
                        woct -= 1; woct &= 7; //turn by -45deg
                        moved = true;
                    } else {
                        /*pt.x = wp.x+OCT2POINT[woct][0];
                        pt.y = wp.y+OCT2POINT[woct][1];
                        if (isPointIn(pt,wp)) {
                            wp = pt;
                            woct -= 2; woct &= 7; //turn by -90deg
                            moved = true;
                        } else {*/
                            woct += 2; woct &= 7; //turn by +90deg
                        //}
                    }
                }
                return moved;
        }
    }
    
    void removeSingleUnassigned() {
        int reg, k;
        int neighs[8][2] = {{1,0},{1,1},{0,1},{-1,-1},  {-1,0},{-1,-1},{0,-1},{1,-1}};
        for (int x=0;x<map_divided.info.width; x++) {
            for (int y=0;y<map_divided.info.height; y++) {
                k = x + y*map_divided.info.width;
                if (map_divided.data[k]==255 && cmap.getTrackStarId(x,y)>=0) {
                    //found unassigned vertex
                    for (int n=0; n<8; n++) {//find neighbors with assigned vertices
                        reg = map_divided.data[x+neighs[n][0] + (y+neighs[n][1])*map_divided.info.width];
                        if (reg != 255) {
                            probs_actual[reg] += pdf->data[k];
                            map_divided.data[k] = reg;
                            break;
                        }
                    }
                }
            }
        }
    }
    
    #ifdef DEBUG_DIVIDER
        void publishBoundaryPose() {
            //ROS_WARN("Boundary pose: (%d,%d), oct=%d", wp.x, wp.y, woct);
            geometry_msgs::PoseStamped msg;
            updatePose(msg, wp.x, wp.y);
            msg.header.frame_id="/map";
            msg.pose.orientation = tf::createQuaternionMsgFromYaw((double)(woct)*M_PI/4);
            pub_debug_pose_border.publish(msg);
        }
    #endif
    
};



}
