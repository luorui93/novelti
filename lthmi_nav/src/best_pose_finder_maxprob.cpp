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


Point BestPoseFinder::findMaxprobPose(lthmi_nav::FloatMapConstPtr pdf) {
    double prob, maxprob = 0.0;
    Point pt;
    for (int x=ra_min.x; x<ra_max.x; x++) {
        for (int y=ra_min.y; y<ra_max.y; y++) {
            prob = pdf->data[x+r2a.x, (y+r2a.y)*pdf->info.width];
            if (prob > maxprob) {
                pt(x,y);
                maxprob = prob;
            }
        }
    }
    return pt;
}

Point BestPoseFinder::findTomaxprobPose(lthmi_nav::FloatMapConstPtr pdf) {
    double d, prob, maxprob = 0.0;
    
    Point maxprob_pt;
    for (int x=0.x; x<pdf->info.width; x++) {
        for (int y=0; y<pdf->info.height; y++) {
            prob = pdf->data[x, y*pdf->info.width];
            if (prob > maxprob) {
                pt(x,y);
                maxprob = prob;
            }
        }
    }
    
    
    Point pt;
    for (int x=ra_min.x; x<ra_max.x; x++) {
        for (int y=ra_min.y; y<ra_max.y; y++) {
            d = sqrt(pow(x+r2a.x-maxprob.x, 2), pow(y+r2a.y-maxprob.y));
            if (d < dmin) {
                pt(x,y);
                dmin = d;
            }
        }
    }
    return pt;
}


}
