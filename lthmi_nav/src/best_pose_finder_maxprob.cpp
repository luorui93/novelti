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



class MaxprobPoseFinder :  public BestPoseFinder {
public:
    bool toMaxprob;
    
    MaxprobPoseFinder(bool toMaxprob1) {
        toMaxprob = toMaxprob1;
    }

    Point findBestPose(lthmi_nav::FloatMapConstPtr pdf) {
        return toMaxprob
            ? findTomaxprobPose(pdf) 
            : findMaxprobPose(pdf);
    }
    
    Point findMaxprobPose(lthmi_nav::FloatMapConstPtr pdf) {
        double prob, maxprob = 0.0;
        Point pt;
        for (int x=ra_min.x; x<ra_max.x; x++) {
            for (int y=ra_min.y; y<ra_max.y; y++) {
                prob = pdf->data[x+r2a.x, (y+r2a.y)*pdf->info.width];
                if (prob > maxprob) {
                    pt.x=x; pt.y=y;
                    maxprob = prob;
                }
            }
        }
        return pt;
    }

    Point findTomaxprobPose(lthmi_nav::FloatMapConstPtr pdf) {
        double d, prob, maxprob = 0.0;
        
        Point maxprob_pt;
        for (int x=0; x<pdf->info.width; x++) {
            for (int y=0; y<pdf->info.height; y++) {
                prob = pdf->data[x, y*pdf->info.width];
                if (prob > maxprob) {
                    maxprob_pt.x=x; maxprob_pt.y=y;
                    maxprob = prob;
                }
            }
        }
        Point pt;
        double dmin=1.0;
        int x2, y2;
        for (int x=ra_min.x; x<ra_max.x; x++) {
            for (int y=ra_min.y; y<ra_max.y; y++) {
                x2 = x+r2a.x-maxprob_pt.x;
                y2 = y+r2a.y-maxprob_pt.y;
                d = sqrt(x2*x2+y2*y2);
                if (d < dmin) {
                    pt.x=x; pt.y=y;
                    dmin = d;
                }
            }
        }
        return pt;
    }

};
