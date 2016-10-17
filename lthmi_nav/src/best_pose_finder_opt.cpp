#include <lthmi_nav/best_pose_finder_cog.h>
#include <random>


using namespace lthmi_nav;


class OptPoseFinder :  public CogPoseFinder, public CWave2Processor {
public:
    const int NEIGHBOURS[8][2] = {{1,0}, {1,1}, {0,1}, {-1,1}, {-1,0}, {-1,-1}, {0,-1}, {1,-1}};
    bool local;
    int glob_max_attempts;
    std::default_random_engine generator;
    double meanDist;
    lthmi_nav::FloatMapConstPtr pdf;
    
    
    OptPoseFinder(bool isLocal) {
        local = isLocal;
        if (!local)
            node.param<int>("glob_max_attempts", glob_max_attempts, 25);
    }
    
    double calcMeanDist(Point point) {
        //input (point) wrt to reach area
        meanDist = 0.0;
        CWave2 cw(cmap);
        cw.setProcessor(this);
        cw.calc(Point(point.x+r2a.x, point.y+r2a.y));
        cmap.clearDist();
        return meanDist;
    }
    
    bool getMeanDist(Point point, double* dist) {
        // input (point) wrt to reach area
        // returns true if the mean distance has not been previously calculated for point (= reach_area had REACH_AREA_UNASSIGNED value for vertex point)
        int k = point.x + point.y*reach_area.info.width;
        bool was_unassigned = (reach_area.data[k]==REACH_AREA_UNASSIGNED);
        if (was_unassigned) { //means not visited
            n_unassigned--;
            reach_area.data[k] = calcMeanDist(point);
        }
        *dist = reach_area.data[k];
        return was_unassigned;
    }
    
    double slideToLocalMin() {
        /*  moves p to local minimum
            * returns mean distance in optimal pose */
        double dist, min_dist;
        getMeanDist(pt, &min_dist);
        int best_nb;
        while (true) {
            best_nb = -1; //best neighbour
            for (int nb=0; nb<8; nb++) { //iterate over neighbours
                pt.x += NEIGHBOURS[nb][0];  pt.y += NEIGHBOURS[nb][1];
                if (getMeanDist(pt, &dist) && dist < min_dist) {
                    min_dist = dist;
                    best_nb = nb;
                }
            }
            if (best_nb>=0) {
                pt.x+=NEIGHBOURS[best_nb][0]; pt.y+=NEIGHBOURS[best_nb][1];
            } else {
                break;
            }
        }
        return min_dist;
    }
    
    void findLocalOptPose() {
        findCogOnPdf(pdf);
        moveToClosestOnMap(pdf);
        moveToClosestInReachAreaObst();
        slideToLocalMin();
    }
    
    void genStartVertex() {
        std::uniform_int_distribution<int> rand_distribution(0,n_unassigned);
        int n = rand_distribution(generator);
        for (int x=ra_min.x; x<ra_max.x; x++)
            for (int y=ra_min.y; y<ra_max.y; y++)
                if (reach_area.data[x + y*reach_area.info.width] == REACH_AREA_UNASSIGNED) {
                    n--;
                    if (n<0) {
                        pt = Point(x,y);
                        return;
                    }
                }
    }
    
    void findGlobalOptPose() {
        double dist, min_dist = std::numeric_limits<double>::max();
        Point opt_pt;
        for (int k=0; k<glob_max_attempts; k++) {
            genStartVertex();
            dist = slideToLocalMin(); // wrt reach area frame
            if (dist<min_dist) {
                min_dist = dist;
                opt_pt = pt;
            }
        }
        pt = opt_pt;
    }
    
    void findBestPose(lthmi_nav::FloatMapConstPtr pdf1) {
        pdf = pdf1;
        if (local)
            findLocalOptPose();
        else
            findGlobalOptPose();
    }
    
    void onSetPointDistance(Star& star, OctPoint& op, bool is_nbp, Point& p, int old_dist, int new_dist) {
        if (old_dist != MAP_POINT_UNEXPLORED)
            new_dist-=old_dist;
        meanDist += pdf->data[p.x + p.y*pdf->info.width] * new_dist;
    };
    void onDistanceCorrection(Star& parent_star, Point& p, int old_dist, int new_dist) {
        if (old_dist != MAP_POINT_UNEXPLORED)
            new_dist-=old_dist;
        meanDist += pdf->data[p.x + p.y*pdf->info.width] * new_dist;
    };
};
