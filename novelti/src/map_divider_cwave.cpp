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

#include <novelti/map_divider.h>
#include <CompoundMap.h>
#include <CWave2.h>

#ifdef DEBUG_DIVIDER
#include <tf/transform_datatypes.h>
#endif

#include <cmath>
#include <stack>

using namespace cwave;

namespace novelti
{

class EquidistMapDividerCWaveProc : public CWave2Processor
{
  public:
    MapDivider &mdiv_;

    EquidistMapDividerCWaveProc(MapDivider &mdiv) : mdiv_(mdiv) {
    }

    void onInitSource(Point &pt) {
        mdiv_.markVertex(pt.x, pt.y);
    }

    void onSetPointDistance(Star &star, OctPoint &op, bool is_nbp, Point &pt, int old_dist, int new_dist) {
        mdiv_.markVertex(pt.x, pt.y);
    }

    void onDistanceCorrection(Star &ps, Point &pt, int old_dist, int new_dist) {
        mdiv_.markVertex(pt.x, pt.y);
    };
};

class CWaveProcPassOne;
class CWaveProcPassTwo;

class CWaveMapDivider : public MapDivider, public CWave2Processor
{
  public:
    enum DivMethod
    {
        EQUIDIST,
        EXTREMAL,
        EXTREDIST,
        NEARCOG_EXTREMAL
    };

    DivMethod method_;
    bool even_;
    CompoundMap cmap_;

#ifdef DEBUG_DIVIDER
    ros::Publisher pub_debug_track_map;
#endif

    CWaveMapDivider(DivMethod method);
    CWaveMapDivider(DivMethod method, string paramPrefix);
    void startExp(novelti::StartExperiment::Request &req);
    void divide();
    void preprocessOverlappingStars(CWaveProcPassOne &procOne, CWaveProcPassTwo &procTwo);
    void divideNearCogByExtremals();
    void divideByExtremals();
    void divideByEquidists();
    void iterateStarRegion(CWaveProcPassTwo &procTwo, int s);
    bool isChildFirst(int base_star_id, int star_k, int child_k, int next_k);

    Point findCogOnPdf();
    Point moveToClosestOnMap(Point pt);
};

class CWaveProcPassOne : public CWave2Processor
{
  public:
    CompoundMap &cmap_;
    vector<int> star_map_;
    vector<int> verts_per_star_;
    #ifdef DEBUG_DIVIDER
        CWaveMapDivider &d_;
    #endif

    CWaveProcPassOne(CompoundMap &cmap, CWaveMapDivider &d) : 
    cmap_(cmap)
#ifdef DEBUG_DIVIDER              
    ,
    d_(d)
#endif
    {
        star_map_ = vector<int>(cmap.width() * cmap.height(), -1);
    }

    void onInitSource(Point &p) {
        verts_per_star_.push_back(0);
        //star_map_[p.x + p.y*cmap_.width()] = 0;
    };

    void onAddStar(Star &parent_star, Star &s) {
        //if (s.id==15 || s.id==16)
        //    ROS_INFO(">>>>>>>>>>>>>>>>>>> parent_star_id=%d",parent_star.id);
        //star_map_[s.c.x + s.c.y*cmap_.width()] = s.id;
        verts_per_star_.push_back(0);
    };

    void procVertex(Star &star, Point &p, int old_dist) {
        #ifdef DEBUG_DIVIDER
        //d_.publishDebugPose(p.x,p.y);
        #endif
        if (old_dist == MAP_POINT_UNEXPLORED) {
            verts_per_star_[star.id]++;
        }
        else {
            int old_star_id = cmap_.getTrackStarId(p.x, p.y);
            verts_per_star_[star.id]++;
            //verts_per_star_[old_star_id]--;
        }
    }

    void onSetPointDistance(Star &star, OctPoint &op, bool is_nbp, Point &p, int old_dist, int new_dist) {
        procVertex(star, p, old_dist);
    };

    void onDistanceCorrection(Star &parent_star, Point &p, int old_dist, int new_dist) {
        procVertex(parent_star, p, old_dist);
    }

    void updateStarMap() {
        TrackStar star;
        for (int s = cmap_.track_stars.size() - 1; s >= 0; s--)
        {
            star = cmap_.track_stars[s];
            star_map_[star.x + star.y * cmap_.width()] = s;
        }
    }
};

struct SortVertex
{
    int k;
    float angle;
    int idist;

    bool operator<(const SortVertex &vs) const
    {
        if (this->angle < vs.angle)
        {
            return true;
        }
        else if (this->angle == vs.angle)
        {
            return (this->idist < vs.idist);
        }
        else
        {
            return false;
        }
    }
};

class CWaveProcPassTwo : public CWave2Processor
{
  public:
    CompoundMap &cmap_;
    vector<int> indexes_;

    vector<vector<SortVertex>> star_vertices_;
    CWaveProcPassOne &procOne_;
    vector<int> &track_map_;

    CWaveProcPassTwo(CompoundMap &cmap, CWaveProcPassOne &procOne, vector<int> &track_map) : cmap_(cmap),
                                                                                             procOne_(procOne),
                                                                                             track_map_(track_map)
    { //star_map_(142+43*cmap_.width())
        star_vertices_ = vector<vector<SortVertex>>(procOne.verts_per_star_.size(), vector<SortVertex>());
        for (int s = 0; s < procOne.verts_per_star_.size(); s++)
        {
            star_vertices_[s] = vector<SortVertex>(procOne.verts_per_star_[s]);
        }
        indexes_ = vector<int>(procOne.verts_per_star_.size(), 0);
    }

    /*~CWaveProcPassTwo() {
        star_vertices_.~vector<vector<SortVertex>>();
        indexes_.~vector<int>();
    }*/

    float calcPseudoangle_0to4(int dx, int dy) { //return 0..4
        //from: http://stackoverflow.com/a/16542043/5787022
        float p = (float)dy / (abs(dx) + abs(dy));
        if (dx < 0)
            p = 2 - p;
        else if (dy < 0)
            p = 4 + p;
        return p;
    }

    float calcPseudoangle_m2to2(int dx, int dy) { //return -2..2
        //from: http://stackoverflow.com/a/16542043/5787022
        float p = (float)dx / (abs(dx) + abs(dy)); //-1 .. 1 increasing with x
        if (dy < 0)
            return p - 1; //-2 .. 0 increasing with x
        else
            return 1 - p; // 0 .. 2 decreasing with x
    }

    void procVertex(Star &star, Point &p, int old_dist) {
        #ifdef DEBUG_DIVIDER
                procOne_.d_.publishDebugPose(p.x, p.y);
        #endif
        int k = p.x + p.y * cmap_.width();
        if (star.id == track_map_[k]) {
            int dx = p.x - star.c.x;
            int dy = p.y - star.c.y;
            /*if (star.id!=0 && dx>0) {
                std::swap(dx,dy);
                dx = -dx;
            }*/
            // float pangle = (star.id != 0 && (dy >= 0 && dx > 0))
            //                    ? calcPseudoangle_m2to2(dx, dy)
            //                    : calcPseudoangle_0to4(dx, dy);

            float pangle = calcPseudoangle_0to4(dx, dy);
            if (dy==0 && dx>0 && star.id!=0) {
                int parent_star_id = track_map_[star.c.x+star.c.y * cmap_.width()];
                if (cmap_.track_stars[parent_star_id].y>star.c.y)
                    pangle = 6;
            }
            ROS_DEBUG("s=%d, p=(%d,%d), (dx,dy)=(%d,%d) -> pangle=%f", star.id, p.x, p.y, dx, dy, pangle);
            star_vertices_[star.id][indexes_[star.id]] = {k, pangle, star.idist};
            if (indexes_[star.id] >= star_vertices_[star.id].size())
                ROS_INFO("--------------------------- star.id=%d, index=%d, p=(%d,%d)", star.id, indexes_[star.id], p.x, p.y);
            indexes_[star.id]++;
        }
    }

    void onSetPointDistance(Star &star, OctPoint &op, bool is_nbp, Point &p, int old_dist, int new_dist) {
        procVertex(star, p, old_dist);
    };

    void onDistanceCorrection(Star &parent_star, Point &p, int old_dist, int new_dist) {
        if (old_dist == MAP_POINT_UNEXPLORED || cmap_.getTrackStarId(p.x, p.y) != parent_star.id)
            procVertex(parent_star, p, old_dist);
    }

    void sortRegions() {
        for (int s = 0; s < star_vertices_.size(); s++)
        {
            star_vertices_[s].resize(indexes_[s]);
            std::sort(star_vertices_[s].begin(), star_vertices_[s].end());
        }
    }
};

CWaveMapDivider::CWaveMapDivider(DivMethod method) : MapDivider(),
                                                     method_(method)
{
    even_ = true;
#ifdef DEBUG_DIVIDER
    pub_debug_track_map = node.advertise<IntMap>("/debug_track_map", 1, true); //not latched
#endif
}

CWaveMapDivider::CWaveMapDivider(DivMethod method, string paramPrefix) : MapDivider(paramPrefix),
                                                                         method_(method)
{
    even_ = true;
#ifdef DEBUG_DIVIDER
    pub_debug_track_map = node.advertise<IntMap>("/debug_track_map", 1, true); //not latched
#endif
}

void CWaveMapDivider::startExp(novelti::StartExperiment::Request &req) {
    new (&cmap_) CompoundMap(req.map.info.width, req.map.info.height);
    for (int x = 0; x < req.map.info.width; x++)
        for (int y = 0; y < req.map.info.height; y++)
            if (req.map.data[x + y * req.map.info.width] == 0)
                cmap_.setPixel(x, y, FREED); //free
    MapDivider::startExp(req);
}

void CWaveMapDivider::divide() {
    switch (method_)
    {
    case EQUIDIST:
        divideByEquidists();
        break;
    case EXTREMAL:
        divideByExtremals();
        break;
    case EXTREDIST:
        if (even_)
            divideByExtremals();
        else
            divideByEquidists();
        even_ = !even_;
    case NEARCOG_EXTREMAL:
        divideNearCogByExtremals();
        break;
    }
}

void CWaveMapDivider::preprocessOverlappingStars(CWaveProcPassOne &procOne, CWaveProcPassTwo &procTwo) {
    procOne.star_map_ = vector<int>(cmap_.width() * cmap_.height(), -1);
    TrackStar star;
    int k, cur_star_id;
    for (int s = cmap_.track_stars.size() - 1; s >= 0; s--)
    {
        star = cmap_.track_stars[s];
        k = star.x + star.y * cmap_.width();
        cur_star_id = procOne.star_map_[k];
        if (cur_star_id >= 0 && cur_star_id != s)
        {
            procTwo.star_vertices_[cur_star_id].insert(
                procTwo.star_vertices_[cur_star_id].end(),
                procTwo.star_vertices_[s].begin(),
                procTwo.star_vertices_[s].end());
        }
        else
        {
            procOne.star_map_[k] = s;
        }
    }
}

void CWaveMapDivider::divideByExtremals() {
    CWave2 cw(cmap_);
    CWaveProcPassOne procOne(cmap_, *this); //broke in single node mode
    cw.setProcessor(&procOne);
    cw.calc(pt_best);
#ifdef DEBUG_DIVIDER
    IntMap track_map_msg = map_divided;
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

    preprocessOverlappingStars(procOne, procTwo);

    procTwo.sortRegions();
    markVertex(pt_best.x, pt_best.y);
    iterateStarRegion(procTwo, 0);
    cmap_.clearDist();
}

void CWaveMapDivider::divideNearCogByExtremals() {
    Point pt_cog = findCogOnPdf();
    Point pt_nearcog = moveToClosestOnMap(pt_cog);

    CWave2 cw(cmap_);
    CWaveProcPassOne procOne(cmap_, *this);
    cw.setProcessor(&procOne);
    cw.calc(pt_nearcog);
#ifdef DEBUG_DIVIDER
    IntMap track_map_msg = map_divided;
    track_map_msg.data = cmap_.track_map;
    pub_debug_track_map.publish(track_map_msg);
#endif
    vector<int> track_map = cmap_.track_map;
    procOne.updateStarMap();

    cmap_.clearDist();

    CWave2 cw2(cmap_);
    CWaveProcPassTwo procTwo(cmap_, procOne, track_map);
    cw2.setProcessor(&procTwo);
    cw2.calc(pt_nearcog);

    preprocessOverlappingStars(procOne, procTwo);

    procTwo.sortRegions();
    markVertex(pt_nearcog.x, pt_nearcog.y);
    iterateStarRegion(procTwo, 0);
    cmap_.clearDist();
}

Point CWaveMapDivider::findCogOnPdf() {
    //returns wrt to map
    float p, xsum = 0.0, ysum = 0.0, psum = 0.0;
    for (int x = 0; x < pdf->info.width; x++)
    {
        for (int y = 0; y < pdf->info.height; y++)
        {
            p = pdf->data[x + y * pdf->info.width];
            if (p > 0.0)
            {
                xsum += p * x;
                ysum += p * y;
                psum += p;
            }
        }
    }
    return Point(int(round(xsum / psum)), int(round(ysum / psum))); //wrt to map
}

Point CWaveMapDivider::moveToClosestOnMap(Point pt) {
    //input (pt)  wrt to map
    //output (pt) wrt to reach_area
    Point out;
    double d, dmin = std::numeric_limits<double>::max();
    for (int x = 0; x < pdf->info.width; x++)
    {
        for (int y = 0; y < pdf->info.height; y++)
        {
            if (pdf->data[x + y * pdf->info.width] >= 0.0)
            {
                d = sqrt((x - pt.x) * (x - pt.x) + (y - pt.y) * (y - pt.y));
                if (d < dmin)
                {
                    out.x = x;
                    out.y = y;
                    dmin = d;
                }
            }
        }
    }
    return out;
}

bool CWaveMapDivider::isChildFirst(int base_star_id, int star_k, int child_k, int next_k) {
    TrackStar base_star = cmap_.getTrackStar(base_star_id);
    int sx = star_k % cmap_.width();
    int sy = star_k / cmap_.width();
    int cx = child_k % cmap_.width() - sx;
    int cy = child_k / cmap_.width() - sy;
    int nx = next_k % cmap_.width() - base_star.x;
    int ny = next_k / cmap_.width() - base_star.y;
    return cx * ny - cy * nx > 0;
}

void CWaveMapDivider::iterateStarRegion(CWaveProcPassTwo &procTwo, int s) {
    int k;
    std::stack<int> left_stars;
    SortVertex v;
    float cur_pangle;
    for (int i = 0; i < procTwo.star_vertices_[s].size(); i++)
    {
        v = procTwo.star_vertices_[s][i];
        if (v.angle != cur_pangle)
        {
            while (!left_stars.empty())
            {
                iterateStarRegion(procTwo, left_stars.top());
                left_stars.pop();
            }
        }
        cur_pangle = v.angle;
        markVertex(v.k);

#ifdef DEBUG_DIVIDER
        int x = v.k % cmap_.width();
        int y = v.k / cmap_.width();
        publishDebugPose(x, y);
        pub_debug_map_div.publish(map_divided);
#endif
        int child_star_id = procTwo.procOne_.star_map_[v.k];
        if (child_star_id >= 0)
        {
            if (i == procTwo.star_vertices_[s].size() - 1 || (procTwo.star_vertices_[child_star_id].size() > 0 &&
                                                              isChildFirst(s, v.k, procTwo.star_vertices_[child_star_id][0].k, procTwo.star_vertices_[s][i + 1].k)))
                iterateStarRegion(procTwo, child_star_id);
            else
                left_stars.push(child_star_id);
        }
    }
    while (!left_stars.empty())
    {
        iterateStarRegion(procTwo, left_stars.top());
        left_stars.pop();
    }
}
//};

void CWaveMapDivider::divideByEquidists() {
    EquidistMapDividerCWaveProc proc(*this);
    CWave2 cw(cmap_);
    cw.setProcessor(&proc);
    cw.calc(pt_best);
    cmap_.clearDist();
}
}
