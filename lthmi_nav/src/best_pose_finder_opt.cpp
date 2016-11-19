#include <lthmi_nav/best_pose_finder_quasi_opt.h>
#include <random>


namespace lthmi_nav {


class OptPoseFinder :  public QuasiOptPoseFinder, public CWave2Processor {
public:
    enum Method { COG2LOPT, RAMAXPROB2LOPT, MAXPROB2LOPT, GOPT };

    const int NEIGHBOURS[8][2] = {{1,0}, {1,1}, {0,1}, {-1,1}, {-1,0}, {-1,-1}, {0,-1}, {1,-1}};
    int glob_max_attempts;
    std::default_random_engine generator;
    double meanDist;
    lthmi_nav::FloatMapConstPtr pdf;
    Method method_;
    
    OptPoseFinder(Method method) :
        QuasiOptPoseFinder(RA_MAXPROB),
        method_(method)
    {
        if (method_ == GOPT)
            node.param<int>("glob_max_attempts", glob_max_attempts, 25);
    }
    
    double calcAvgDist(Point point) {
        //input (point) wrt to reach area
        meanDist = 0.0;
        CWave2 cw(cmap);
        cw.setProcessor(this);
        cw.calc(Point(point.x+r2a.x, point.y+r2a.y));
        ROS_DEBUG("%s: calculated avgDist(%d,%d)=%f", getName().c_str(), point.x, point.y, meanDist);
        cmap.clearDist();
        return meanDist;
    }
    
    bool getMeanDist(Point point, double& dist) {
        // input (point) wrt to reach area
        // returns true if the mean distance has not been previously calculated for point (= reach_area had REACH_AREA_UNASSIGNED value for vertex point)
        int k = point.x + point.y*reach_area.info.width;
        bool was_unassigned = (reach_area.data[k]==REACH_AREA_UNASSIGNED);
        if (was_unassigned) { //means not visited
            n_unassigned--;
            reach_area.data[k] = calcAvgDist(point);
        }
        dist = reach_area.data[k];
        return was_unassigned;
    }
    
    double slideToLocalMin() {
        /*  moves p to local minimum
            * returns mean distance in optimal pose */
        double dist, min_dist;
        getMeanDist(pt, min_dist);
        ROS_DEBUG("%s: Started slideToLocalMin at pt=(%d,%d), avgdist=%f, r2a=(%d,%d)", getName().c_str(), pt.x, pt.y, min_dist, r2a.x, r2a.y);
        int best_nb;
        while (true) {
            best_nb = -1; //best neighbour
            ROS_DEBUG("%s: looking for best neighbour around pt=(%d,%d)", getName().c_str(), pt.x, pt.y);
            for (int nb=0; nb<8; nb++) { //iterate over neighbours
                pt.x += NEIGHBOURS[nb][0];  pt.y += NEIGHBOURS[nb][1];
                if (getMeanDist(pt, dist) && dist>0 && dist < min_dist) {
                    min_dist = dist;
                    best_nb = nb;
                }
                ROS_DEBUG("%s: tried pt=(%d,%d), dist=%f, min_dist=%f", getName().c_str(), pt.x, pt.y, dist, min_dist);
                pt.x -= NEIGHBOURS[nb][0];  pt.y -= NEIGHBOURS[nb][1];
            }
            if (best_nb>=0) {
                pt.x+=NEIGHBOURS[best_nb][0]; pt.y+=NEIGHBOURS[best_nb][1];
                ROS_DEBUG("%s: move to pt=(%d,%d)", getName().c_str(), pt.x, pt.y);
                PUB_DEBUG_POSE(pt.x,pt.y, false);
            } else {
                break;
            }
        }
        return min_dist;
    }
    
    double findNearCog2LocalOptPose() {
        findCogOnPdf(pdf);              PUB_DEBUG_POSE(pt.x,pt.y, true);
        moveToClosestOnMap(pdf);        PUB_DEBUG_POSE(pt.x,pt.y, true);
        moveToClosestInReachAreaObst(); PUB_DEBUG_POSE(pt.x,pt.y, false);
        return slideToLocalMin();
    }
    
    double findRaMaxprob2localOptPose() {
        findMaxprobInReachArea(pdf);    PUB_DEBUG_POSE(pt.x,pt.y, false);
        return slideToLocalMin();
    }
            
    double findMaxprob2localOptPose() {
        findMaxprobInPdf(pdf);          PUB_DEBUG_POSE(pt.x,pt.y, true);
        moveToClosestInReachAreaObst(); PUB_DEBUG_POSE(pt.x,pt.y, false);
        return slideToLocalMin();
    }
    
    void genStartVertex() {
        std::uniform_int_distribution<int> rand_distribution(0,n_unassigned-1);
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
        
        dist = findNearCog2LocalOptPose();
        if (dist<min_dist) {
            min_dist = dist;
            opt_pt = pt;
        }
        dist = findRaMaxprob2localOptPose();
        if (dist<min_dist) {
            min_dist = dist;
            opt_pt = pt;
        }
        dist = findMaxprob2localOptPose();
        if (dist<min_dist) {
            min_dist = dist;
            opt_pt = pt;
        }
        
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
        pdf = pdf1; //COG2LOPT, RAMAXPROB2LOPT, MAXPROB2LOPT, GOPT
        switch (method_) {
            case COG2LOPT: 
                findNearCog2LocalOptPose(); break;
            case MAXPROB2LOPT:
                findMaxprob2localOptPose(); break;
            case RAMAXPROB2LOPT:
                findRaMaxprob2localOptPose(); break;
            case GOPT:
                findGlobalOptPose(); break;
        }
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

}