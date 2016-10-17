#include <lthmi_nav/best_pose_finder.h>


using namespace lthmi_nav;



class MaxprobPoseFinder :  public BestPoseFinder {
public:
    bool useEuqlidDist;
    bool raMaxProb;
    
    
    MaxprobPoseFinder() :
        BestPoseFinder()
    {
        raMaxProb = true;
    }

    MaxprobPoseFinder(bool useEuqlidDist) :
        BestPoseFinder()
    {
        raMaxProb = false;
        useEuqlidDist = useEuqlidDist;
    }
    
    void findMaxprobInReachArea(lthmi_nav::FloatMapConstPtr pdf) {
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

    void findMaxprobInPdf(lthmi_nav::FloatMapConstPtr pdf) {
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

    void findBestPose(lthmi_nav::FloatMapConstPtr pdf) {
        //output (pt) wrt to reach_area
        if (raMaxProb) {
            findMaxprobInReachArea(pdf);
        } else { 
            findMaxprobInPdf(pdf);
            if (useEuqlidDist)
                moveToClosestInReachAreaEuq();
            else 
                moveToClosestInReachAreaObst();
        } 
    }
};
