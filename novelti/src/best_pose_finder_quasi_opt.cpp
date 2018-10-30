#include <novelti/best_pose_finder_quasi_opt.h>

namespace novelti {

    QuasiOptPoseFinder::QuasiOptPoseFinder(Method method) :
        BestPoseFinder(),
        method_(method)
    { }

    QuasiOptPoseFinder::QuasiOptPoseFinder(Method method, string paramPrefix) :
        BestPoseFinder(paramPrefix),
        method_(method)
    {}

    
    void QuasiOptPoseFinder::findCogOnPdf() { 
        //returns wrt to map
        float p, xsum=0.0, ysum=0.0, psum=0.0;
        for (int x=0; x<pdf_->info.width; x++) {
            for (int y=0; y<pdf_->info.height; y++) {
                p = pdf_->data[x + y*pdf_->info.width];
                if (p>0.0) {
                    xsum += p*x;
                    ysum += p*y;
                    psum += p;
                }
            }
        }
        pt = Point(int(round(xsum/psum)), int(round(ysum/psum))); //wrt to map
    }

    
    void QuasiOptPoseFinder::findMaxprobInReachArea() {
        //output (pt) wrt to reach_area
        double prob, maxprob = 0.0;
        for (int x=ra_min.x; x<ra_max.x; x++) {
            for (int y=ra_min.y; y<ra_max.y; y++) {
                if (reach_area.data[x+y*reach_area.info.width] != REACH_AREA_UNREACHABLE) {
                    prob = pdf_->data[x+r2a.x + (y+r2a.y)*pdf_->info.width];
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

    
    void QuasiOptPoseFinder::findMaxprobInPdf() {
        //output (pt) wrt to pdf
        double d, prob, maxprob = 0.0;
        for (int x=0; x<pdf_->info.width; x++) {
            for (int y=0; y<pdf_->info.height; y++) {
                prob = pdf_->data[x + y*pdf_->info.width];
                if (prob > maxprob) {
                    pt.x=x; pt.y=y;
                    maxprob = prob;
                }
            }
        }
    }
    
    
    void QuasiOptPoseFinder::findBestPose() {
        switch(method_) {
            case RA_MAXPROB:
                findMaxprobInReachArea();       PUB_DEBUG_POSE(pt.x,pt.y, false); //false == wrt RA
                break;
            case MAXPROB_EUC:
                findMaxprobInPdf();             PUB_DEBUG_POSE(pt.x,pt.y, true);
                moveToClosestInReachAreaEuc();
                break;
            case MAXPROB_OBST:
                findMaxprobInPdf();             PUB_DEBUG_POSE(pt.x,pt.y, true);
                moveToClosestInReachAreaObst(); 
                break;
            case COG_EUC:
                findCogOnPdf();                 PUB_DEBUG_POSE(pt.x,pt.y, true);
                moveToClosestInReachAreaEuc();
                break;
            case NEARCOG_EUC:
                findCogOnPdf();                 PUB_DEBUG_POSE(pt.x,pt.y, true);
                moveToClosestOnMap();           PUB_DEBUG_POSE(pt.x,pt.y, true);
                moveToClosestInReachAreaEuc();
                break;
            case NEARCOG_OBST:
                findCogOnPdf();                 PUB_DEBUG_POSE(pt.x,pt.y, true);
                moveToClosestOnMap();           PUB_DEBUG_POSE(pt.x,pt.y, true);
                moveToClosestInReachAreaObst();
                break;
        }
    }
};
