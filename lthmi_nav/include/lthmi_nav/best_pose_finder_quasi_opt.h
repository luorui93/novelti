#ifndef LTHMI_NAV_BEST_POSE_FINDER_QUASI_OPT
#define LTHMI_NAV_BEST_POSE_FINDER_QUASI_OPT

#include <lthmi_nav/best_pose_finder.h>

namespace lthmi_nav{

class QuasiOptPoseFinder :  public BestPoseFinder {
public:
    enum Method { RA_MAXPROB, MAXPROB_EUC, MAXPROB_OBST, COG_EUC, NEARCOG_EUC, NEARCOG_OBST };
    
    Method method_;
    
    QuasiOptPoseFinder(Method method); 
    void findCogOnPdf(lthmi_nav::FloatMapConstPtr pdf);
    void findBestPose(lthmi_nav::FloatMapConstPtr pdf);
    void findMaxprobInReachArea(lthmi_nav::FloatMapConstPtr pdf);
    void findMaxprobInPdf(lthmi_nav::FloatMapConstPtr pdf);
};

}

#endif