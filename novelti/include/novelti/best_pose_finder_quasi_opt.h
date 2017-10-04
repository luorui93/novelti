#ifndef NOVELTI_BEST_POSE_FINDER_QUASI_OPT
#define NOVELTI_BEST_POSE_FINDER_QUASI_OPT

#include <novelti/best_pose_finder.h>

namespace novelti{

class QuasiOptPoseFinder :  public BestPoseFinder {
public:
    enum Method { RA_MAXPROB, MAXPROB_EUC, MAXPROB_OBST, COG_EUC, NEARCOG_EUC, NEARCOG_OBST };
    
    Method method_;
    
    QuasiOptPoseFinder(Method method); 
    void findCogOnPdf(novelti::FloatMapConstPtr pdf);
    void findBestPose(novelti::FloatMapConstPtr pdf);
    void findMaxprobInReachArea(novelti::FloatMapConstPtr pdf);
    void findMaxprobInPdf(novelti::FloatMapConstPtr pdf);
};

}

#endif