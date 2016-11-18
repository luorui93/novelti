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
#include "best_pose_finder_maxprob.cpp"
#include <lthmi_nav/best_pose_finder_cog.h>
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
        bpf = new MaxprobPoseFinder();
    else if (method=="maxprob_euq")  //doesn't guarantee convergance
        bpf = new MaxprobPoseFinder(true);
    else if (method=="maxprob_obst") 
        bpf = new MaxprobPoseFinder(false);
    else if (method=="cog_euq")      //doesn't guarantee convergance
        bpf = new CogPoseFinder();
    else if (method=="nearcog_euq")  //doesn't guarantee convergance
        bpf = new CogPoseFinder(true);
    else if (method=="nearcog_obst") 
        bpf = new CogPoseFinder(false);
    else if (method=="cog2lopt")     
        bpf = new OptPoseFinder(true);
    else if (method=="cog2gopt")     
        bpf = new OptPoseFinder(false);
    else {
         ROS_ERROR("%s: wrong value for 'method' parameter ('%s'), will die now", getName().c_str(), method.c_str());
         return 1;
    }
    bpf->run();
    delete(bpf);
}
