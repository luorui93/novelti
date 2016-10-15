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


using namespace lthmi_nav;

int main(int argc, char **argv) {
    ros::init(argc, argv, "best_pose_finder");
    
    ros::NodeHandle n("~");
    std::string method = "maxprob";
    n.getParam("method", method);
    ROS_INFO("%s: started. method=%s", getName().c_str(), method.c_str());
    
    BestPoseFinder* bpf = nullptr;
    if      (method=="maxprob")    bpf = new MaxprobPoseFinder(false);
    else if (method=="tomaxprob")  bpf = new MaxprobPoseFinder(true);
// "opt")     
// "localopt")
// "maxprob") 
// "tomaxprob"
// "cog")     
// "cog2")    
// "cog2lopt")
// "all3cog") 
// "all3lopt")
// "all3gopt")
     else { 
         ROS_ERROR("%s: wrong value for 'method' parameter ('%s'), will die now", getName().c_str(), method.c_str());
         return 1;
    }
    bpf->run();
    delete(bpf);
}
