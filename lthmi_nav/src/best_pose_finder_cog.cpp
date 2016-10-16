#include <lthmi_nav/best_pose_finder.h>


using namespace lthmi_nav;



class CogPoseFinder :  public BestPoseFinder {
public:
    bool useEuqlidDist;
    bool nearcog;
    
    
    CogPoseFinder() {
        nearcog = false;
    }

    CogPoseFinder(bool useEuqlidDist) {
        nearcog = true;
        useEuqlidDist = useEuqlidDist;
    }
    
    Point findCogOnPdf(lthmi_nav::FloatMapConstPtr pdf) { //returns wrt to map
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
        return Point(int(round(xsum/psum)), int(round(ysum/psum))); //wrt to map
    }

    Point findBestPose(lthmi_nav::FloatMapConstPtr pdf) {
        Point cog = findCogOnPdf(pdf);
        if (nearcog) {
            Point pt = findClosestOnMap(pdf, cog);
            if (useEuqlidDist)
                return findClosestInReachAreaEuq(pt);
            else 
                return findClosestInReachAreaObst(pt);
        } else {
            return findClosestInReachAreaEuq(cog);
        } 
    }
};
