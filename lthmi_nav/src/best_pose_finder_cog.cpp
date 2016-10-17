#include <lthmi_nav/best_pose_finder_cog.h>

namespace lthmi_nav {

    CogPoseFinder::CogPoseFinder() {
        nearcog = false;
    }

    CogPoseFinder::CogPoseFinder(bool useEuqlidDist) {
        nearcog = true;
        useEuqlidDist = useEuqlidDist;
    }
    
    void CogPoseFinder::findCogOnPdf(lthmi_nav::FloatMapConstPtr pdf) { //returns wrt to map
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

    void CogPoseFinder::findBestPose(lthmi_nav::FloatMapConstPtr pdf) {
        findCogOnPdf(pdf);
        if (nearcog) {
            moveToClosestOnMap(pdf);
            if (useEuqlidDist)
                moveToClosestInReachAreaEuq();
            else 
                moveToClosestInReachAreaObst();
        } else {
            moveToClosestInReachAreaEuq();
        } 
    }
};
