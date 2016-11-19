//#define DEBUG_POSE_FINDER 1

/*
 subs                                       pubs
           +-------------------------+
           |                         |
           |                         | ---> /pose_best
 /pdf ---> |  node_best_pose_finder  |
           |                         | ---> /reach_area   | debug
           |                         |
           +-------------------------+
                      ^
                      |
                srv: start
                    req:  scene
                    resp: -
*/

#include <lthmi_nav/best_pose_finder.h>
//#include "best_pose_finder_maxprob.cpp"
//#include <lthmi_nav/best_pose_finder_cog.h>
#include <lthmi_nav/best_pose_finder_quasi_opt.h>
#include "best_pose_finder_opt.cpp"


using namespace lthmi_nav;

int main(int argc, char **argv) {
    ros::init(argc, argv, "best_pose_finder");
    
    ros::NodeHandle n("~");
    std::string method = "cog2lopt";
    n.getParam("method", method);
    ROS_INFO("%s: started. method=%s", getName().c_str(), method.c_str());
    
    BestPoseFinder* bpf = nullptr;
    if      (method=="no_move")
        bpf = new BestPoseFinder();
    else if (method=="ra_maxprob")   
        bpf = new QuasiOptPoseFinder(QuasiOptPoseFinder::RA_MAXPROB);
    else if (method=="maxprob_euc")  //doesn't guarantee convergance
        bpf = new QuasiOptPoseFinder(QuasiOptPoseFinder::MAXPROB_EUC);
    else if (method=="maxprob_obst") 
        bpf = new QuasiOptPoseFinder(QuasiOptPoseFinder::MAXPROB_OBST);
    else if (method=="cog_euq")      //doesn't guarantee convergance
        bpf = new QuasiOptPoseFinder(QuasiOptPoseFinder::COG_EUC);
    else if (method=="nearcog_euc")  //doesn't guarantee convergance
        bpf = new QuasiOptPoseFinder(QuasiOptPoseFinder::NEARCOG_EUC);
    else if (method=="nearcog_obst") 
        bpf = new QuasiOptPoseFinder(QuasiOptPoseFinder::NEARCOG_OBST);
    else if (method=="cog2lopt")
        bpf = new OptPoseFinder(OptPoseFinder::COG2LOPT);
    else if (method=="ramaxprob2lopt")
        bpf = new OptPoseFinder(OptPoseFinder::RAMAXPROB2LOPT);
    else if (method=="maxprob2lopt")     
        bpf = new OptPoseFinder(OptPoseFinder::MAXPROB2LOPT);
    else if (method=="gopt")
        bpf = new OptPoseFinder(OptPoseFinder::GOPT);
    else {
         ROS_ERROR("%s: wrong value for 'method' parameter ('%s'), will die now", getName().c_str(), method.c_str());
         return 1;
    }
    bpf->run();
    delete(bpf);
}
