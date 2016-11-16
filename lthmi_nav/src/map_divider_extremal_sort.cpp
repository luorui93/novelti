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

#ifdef DEBUG_DIVIDER
    #include <tf/transform_datatypes.h>
#endif    

#include <cmath>
#include <stack>

using namespace cwave;

namespace lthmi_nav {

class CWaveProcPassTwo;

class ExtremalMapDivider :  public MapDivider, public CWave2Processor {
public:
    CompoundMap cmap_;
    //int region;
    //double prob;
    
    #ifdef DEBUG_DIVIDER
        IntMap track_map_msg;
        ros::Publisher pub_debug_pose_border;
        ros::Publisher pub_debug_track_map;
    #endif
        
    ExtremalMapDivider();
    void start(lthmi_nav::StartExperiment::Request& req);
    void divide();
    void divideByExtremals();
    void iterateStarRegion(CWaveProcPassTwo& procTwo, int s);
    void publishBoundaryPose(int x, int y);
    bool isChildFirst(int star_id, int child_k, int next_k);
};
    


class CWaveProcPassOne : public CWave2Processor {
public:
    
    CompoundMap& cmap_;
    vector <int> star_map_;
    vector <int> verts_per_star_;
    #ifdef DEBUG_DIVIDER
        ExtremalMapDivider& d_;
    #endif
    
    CWaveProcPassOne(CompoundMap& cmap
    , ExtremalMapDivider& d
        
    ) :
        cmap_(cmap)
        #ifdef DEBUG_DIVIDER
            , d_(d)
        #endif
    {
        star_map_ = vector<int>(cmap.width()*cmap.height(), -1);
    }
    
    void onInitSource(Point& p) {
        verts_per_star_.push_back(0);
        //star_map_[p.x + p.y*cmap_.width()] = 0;
    };
    
    void onAddStar(Star& parent_star, Star& s) {
        //star_map_[s.c.x + s.c.y*cmap_.width()] = s.id;
        verts_per_star_.push_back(0);
    };
    
    void procVertex(Star& star, Point& p, int old_dist) {
        #ifdef DEBUG_DIVIDER
            //d_.publishBoundaryPose(p.x,p.y);
        #endif
        if (old_dist == MAP_POINT_UNEXPLORED) {
            verts_per_star_[star.id]++;
        } else {
            int old_star_id = cmap_.getTrackStarId(p.x,p.y);
            verts_per_star_[star.id]++;
            verts_per_star_[old_star_id]--;
        }
    }
    
    void onSetPointDistance(Star& star, OctPoint& op, bool is_nbp, Point& p, int old_dist, int new_dist) { 
        procVertex(star,p, old_dist);
    };
    
    void onDistanceCorrection(Star& parent_star, Point& p, int old_dist, int new_dist) {
        procVertex(parent_star,p, old_dist);
    }
    
    void updateStarMap() {
        TrackStar star;
        for (int s=0;s<cmap_.track_stars.size(); s++) {
            star = cmap_.track_stars[s];
            star_map_[star.x + star.y*cmap_.width()] = s;
        }
    }
};


struct SortVertex {
    int k;
    float angle;
    int idist;
    
    bool operator<(const SortVertex& vs) const {
        if (this->angle < vs.angle) {
            return true;
        } else if (this->angle == vs.angle) {
            return (this->idist < vs.idist);
        } else {
            return false;
        }
    }
};


class CWaveProcPassTwo : public CWave2Processor {
public:
    CompoundMap& cmap_;
    vector<int> indexes_;
    
    vector<vector<SortVertex>> star_vertices_;
    CWaveProcPassOne& procOne_;
    vector<int>& track_map_;
    
    CWaveProcPassTwo(CompoundMap& cmap, CWaveProcPassOne& procOne, vector<int>& track_map) :
        cmap_(cmap),
        procOne_(procOne),
        track_map_(track_map)
    {//star_map_(142+43*cmap_.width())
        star_vertices_ = vector<vector<SortVertex>>(procOne.verts_per_star_.size(), vector<SortVertex>());
        for (int s=0; s<procOne.verts_per_star_.size() ; s++) {
            star_vertices_[s] = vector<SortVertex> (procOne.verts_per_star_[s]);
        }
        indexes_ = vector<int>(procOne.verts_per_star_.size(), 0);
    }
    
    float calcPseudoangle_0to4(int dx, int dy) { //return 0..4
        //from: http://stackoverflow.com/a/16542043/5787022
        float p = (float)dy/(abs(dx)+abs(dy));
        if (dx < 0)
            p = 2 - p;
        else if (dy < 0)
            p = 4 + p;
        return p;
    }
    
    float calcPseudoangle_m2to2(int dx, int dy) { //return -2..2
        //from: http://stackoverflow.com/a/16542043/5787022
        float p = (float)dx/(abs(dx)+abs(dy)); //-1 .. 1 increasing with x
        if (dy < 0)
            return p - 1; //-2 .. 0 increasing with x
        else
            return 1 - p; // 0 .. 2 decreasing with x
    }
    
    void procVertex(Star& star, Point& p, int old_dist) {
        #ifdef DEBUG_DIVIDER
            procOne_.d_.publishBoundaryPose(p.x,p.y);
        #endif
        int k = p.x + p.y*cmap_.width();
        if (star.id == track_map_[k]) {
            int dx = p.x-star.c.x;
            int dy = p.y-star.c.y;
            /*if (star.id!=0 && dx>0) {
                std::swap(dx,dy);
                dx = -dx;
            }*/
            float pangle = (star.id!=0 && (dy>=0 && dx>0))
                ? calcPseudoangle_m2to2(dx, dy)
                : calcPseudoangle_0to4(dx, dy);
                
            ROS_DEBUG("s=%d, p=(%d,%d), (dx,dy)=(%d,%d) -> pangle=%f", star.id, p.x, p.y, dx, dy, pangle);
            star_vertices_[star.id][indexes_[star.id]] = {k, pangle, star.idist};
            //ROS_DEBUG("aaaaaaa");
            indexes_[star.id]++;
        }
    }
    
    void onSetPointDistance(Star& star, OctPoint& op, bool is_nbp, Point& p, int old_dist, int new_dist) {
        procVertex(star, p, old_dist);
    };
    
    void onDistanceCorrection(Star& parent_star, Point& p, int old_dist, int new_dist) {
        if (old_dist==MAP_POINT_UNEXPLORED || cmap_.getTrackStarId(p.x, p.y)!=parent_star.id)
            procVertex(parent_star, p, old_dist);
    }
    
    void sortRegions() {
        for (int s=0; s<star_vertices_.size() ; s++) {
             std::sort(star_vertices_[s].begin(), star_vertices_[s].end());
        }
    }
};

    
ExtremalMapDivider::ExtremalMapDivider() :
        MapDivider() 
    { 
        #ifdef DEBUG_DIVIDER
            pub_debug_pose_border = node.advertise<geometry_msgs::PoseStamped>("/debug_pose_border", 1, true); //not latched
            pub_debug_track_map   = node.advertise<IntMap>("/debug_track_map", 1, true); //not latched
        #endif
    }
    
    void ExtremalMapDivider::start(lthmi_nav::StartExperiment::Request& req) {
        new (&cmap_) CompoundMap(req.map.info.width, req.map.info.height);
        for (int x=0; x<req.map.info.width; x++)
            for (int y=0; y<req.map.info.height; y++)
                if (req.map.data[x + y*req.map.info.width]==0)
                    cmap_.setPixel(x,y, FREED); //free
        MapDivider::start(req);
        #ifdef DEBUG_DIVIDER
            track_map_msg = map_divided; //copy
        #endif
        //track_stars = std::vector<MyStar>(50);
    }
    
    
    void ExtremalMapDivider::divide() {
        divideByExtremals();
    }
    
    void ExtremalMapDivider::divideByExtremals() {
        map_divided.data = std::vector<int>(map_divided.info.width*map_divided.info.height, 255);
        CWave2 cw(cmap_);
        CWaveProcPassOne procOne(cmap_, *this);
        cw.setProcessor(&procOne);
        cw.calc(pt_best);
        #ifdef DEBUG_DIVIDER
            track_map_msg.data = cmap_.track_map;
            pub_debug_track_map.publish(track_map_msg);
        #endif
        vector<int> track_map = cmap_.track_map;
        procOne.updateStarMap();
        
        cmap_.clearDist();
            
        CWave2 cw2(cmap_);
        CWaveProcPassTwo procTwo(cmap_, procOne, track_map);
        cw2.setProcessor(&procTwo);
        cw2.calc(pt_best);
        
        procTwo.sortRegions();
        markVertex(pt_best.x, pt_best.y);
        iterateStarRegion(procTwo, 0);
        cmap_.clearDist();
    }
    
    bool ExtremalMapDivider::isChildFirst(int star_k, int child_k, int next_k) {
        int sx = star_k % cmap_.width();
        int sy = star_k / cmap_.width();
        int cx = child_k % cmap_.width() - sx;
        int cy = child_k / cmap_.width() - sy;
        int nx = next_k % cmap_.width()  - sx;
        int ny = next_k / cmap_.width()  - sy;
        return cx*ny - cy*nx >0;
    }
    
    void ExtremalMapDivider::iterateStarRegion(CWaveProcPassTwo& procTwo, int s) {
        int k;
        std::stack<int> left_stars;
        SortVertex v;
        float cur_pangle;
        for (int i=0; i<procTwo.star_vertices_[s].size() ; i++) {
            v = procTwo.star_vertices_[s][i];
            if (v.angle!=cur_pangle) {
                while (!left_stars.empty()) {
                    iterateStarRegion(procTwo, left_stars.top());
                    left_stars.pop();
                }
            }
            cur_pangle = v.angle;
            markVertex(v.k);

            #ifdef DEBUG_DIVIDER
                int x = v.k % cmap_.width();
                int y = v.k / cmap_.width();
                publishBoundaryPose(x,y);
                pub_map_div.publish(map_divided);
            #endif
            int child_star_id = procTwo.procOne_.star_map_[v.k];
            if (child_star_id >=0) {
                if (i == procTwo.star_vertices_[s].size()-1 || (
                        procTwo.star_vertices_[child_star_id].size()>0 
                            &&
                        isChildFirst(v.k, procTwo.star_vertices_[child_star_id][0].k, procTwo.star_vertices_[s][i+1].k)
                    )
                )
                    iterateStarRegion(procTwo, child_star_id);
                else
                    left_stars.push(child_star_id);
            }
            
        }
        while (!left_stars.empty()) {
            iterateStarRegion(procTwo, left_stars.top());
            left_stars.pop();
        }
    }
    
    #ifdef DEBUG_DIVIDER
        void ExtremalMapDivider::publishBoundaryPose(int x, int y) {
//             //ROS_WARN("Boundary pose: (%d,%d), oct=%d", wp.x, wp.y, woct);
            geometry_msgs::PoseStamped msg;
            updatePose(msg, x, y);
            msg.header.frame_id="/map";
            pub_debug_pose_border.publish(msg);
        }
    #endif
    
//};



}
