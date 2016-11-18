#include <lthmi_nav/best_pose_finder_quasi_opt.h>

namespace lthmi_nav {

    QuasiOptPoseFinder::QuasiOptPoseFinder(Method method) :
        BestPoseFinder(),
        method_(method)
    { }
    
    
    void QuasiOptPoseFinder::findCogOnPdf(lthmi_nav::FloatMapConstPtr pdf) { //returns wrt to map
        float p, xsum=0.0, ysum=0.0, psum=0.0;
        for (int x=0; x<pdf->info.width; x++) {
            for (int y=0; y<pdf->info.height; y++) {
                p = pdf->data[x + y*pdf->info.width];
                if (p>0.0) {
                    xsum += p*x;
                    ysum += p*y;
                    psum += p;
                }
            }
        }
        pt = Point(int(round(xsum/psum)), int(round(ysum/psum))); //wrt to map
    }

    
    void QuasiOptPoseFinder::findMaxprobInReachArea(lthmi_nav::FloatMapConstPtr pdf) {
        //output (pt) wrt to reach_area
        double prob, maxprob = 0.0;
        Point pt;
        for (int x=ra_min.x; x<ra_max.x; x++) {
            for (int y=ra_min.y; y<ra_max.y; y++) {
                if (reach_area.data[x+y*reach_area.info.width] != REACH_AREA_UNREACHABLE) {
                    prob = pdf->data[x+r2a.x + (y+r2a.y)*pdf->info.width];
                    //ROS_INFO("pdf[%d,%d]=%f", x+r2a.x, y+r2a.y, prob);
                    if (prob > maxprob) {
                        pt.x=x; pt.y=y;
                        maxprob = prob;
                        //ROS_INFO("=============pt=[%d,%d], prob=%f", pt.x, pt.y, prob);
                    }
                }
            }
        }
        //ROS_INFO("===========================pdf[%d,%d]=%f", pt.x, pt.y, prob);
    }

    
    void QuasiOptPoseFinder::findMaxprobInPdf(lthmi_nav::FloatMapConstPtr pdf) {
        //output (pt) wrt to pdf
        double d, prob, maxprob = 0.0;
        for (int x=0; x<pdf->info.width; x++) {
            for (int y=0; y<pdf->info.height; y++) {
                prob = pdf->data[x + y*pdf->info.width];
                if (prob > maxprob) {
                    pt.x=x; pt.y=y;
                    maxprob = prob;
                }
            }
        }
    }
    
    
    void QuasiOptPoseFinder::findBestPose(lthmi_nav::FloatMapConstPtr pdf) {
        switch(method_) {
            case RA_MAXPROB:
                findMaxprobInReachArea(pdf);    PUB_DEBUG_POSE(pt.x,pt.y, true);
                break;
            case MAXPROB_EUC:
                findMaxprobInPdf(pdf);          PUB_DEBUG_POSE(pt.x,pt.y, true);
                moveToClosestInReachAreaEuc();  PUB_DEBUG_POSE(pt.x,pt.y, true);
                break;
            case MAXPROB_OBST:
                findMaxprobInPdf(pdf);          PUB_DEBUG_POSE(pt.x,pt.y, true);
                moveToClosestInReachAreaObst(); PUB_DEBUG_POSE(pt.x,pt.y, true);
                break;
            case COG_EUC:
                findCogOnPdf(pdf);              PUB_DEBUG_POSE(pt.x,pt.y, true);
                moveToClosestInReachAreaEuc();
                break;
            case NEARCOG_EUC:
                findCogOnPdf(pdf);              PUB_DEBUG_POSE(pt.x,pt.y, true);
                moveToClosestOnMap(pdf);        PUB_DEBUG_POSE(pt.x,pt.y, true);
                moveToClosestInReachAreaEuc();
                break;
            case NEARCOG_OBST:
                findCogOnPdf(pdf);              PUB_DEBUG_POSE(pt.x,pt.y, true);
                moveToClosestOnMap(pdf);        PUB_DEBUG_POSE(pt.x,pt.y, true);
                moveToClosestInReachAreaEuc();
                break;
        }
    }
};
