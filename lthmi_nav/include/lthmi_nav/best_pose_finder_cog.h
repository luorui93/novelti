#ifndef LTHMI_NAV_BEST_POSE_FINDER_COG
#define LTHMI_NAV_BEST_POSE_FINDER_COG

#include <lthmi_nav/best_pose_finder.h>

namespace lthmi_nav{

class CogPoseFinder :  public BestPoseFinder {
public:
    bool useEuqlidDist;
    bool nearcog;
    
    CogPoseFinder(); 
    CogPoseFinder(bool useEuqlidDist);
    void findCogOnPdf(lthmi_nav::FloatMapConstPtr pdf);
    void findBestPose(lthmi_nav::FloatMapConstPtr pdf);
};

}

#endif