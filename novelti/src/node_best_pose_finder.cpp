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

#include <novelti/best_pose_finder.h>
//#include "best_pose_finder_maxprob.cpp"
//#include <novelti/best_pose_finder_cog.h>
#include <novelti/best_pose_finder_quasi_opt.h>
#include "best_pose_finder_opt.cpp"

using namespace novelti;

class BestPoseFinderNode : public BestPoseFinder, public SynchronizableNode {
public:
    BestPoseFinderNode() {}
    void start(novelti::StartExperiment::Request & req) {
        startExp(req);
    }
    
    void stop() {
        stopExp();
    }
};
class QuasiOptPoseFinderNode: public QuasiOptPoseFinder, public SynchronizableNode {
public:
    QuasiOptPoseFinderNode(QuasiOptPoseFinder::Method method) :
        QuasiOptPoseFinder(method),
        SynchronizableNode() 
        {}
        void start(novelti::StartExperiment::Request &req) {
            startExp(req);
        }

        void stop() {
            stopExp();
        }
};
class OptPoseFinderNode: public OptPoseFinder, public SynchronizableNode {
public:
    OptPoseFinderNode(OptPoseFinder::Method method) :
        OptPoseFinder(method),
        SynchronizableNode()
        {}
    void start(novelti::StartExperiment::Request & req) {
        startExp(req);
    }
    
    void stop() {
        stopExp();
    }
};

using namespace novelti;

int main(int argc, char **argv) {
    ros::init(argc, argv, "best_pose_finder");
    
    ros::NodeHandle n("~");
    std::string method = "cog2lopt";
    if (n.getParam("pos/method", method)) {
        ROS_INFO("%s: started. method=%s", getName().c_str(), method.c_str());
    };
    
    BestPoseFinder* bpf = nullptr;
    if      (method=="no_move")
        bpf = new BestPoseFinderNode();
    else if (method=="ra_maxprob")   
        bpf = new QuasiOptPoseFinderNode(QuasiOptPoseFinder::RA_MAXPROB);
    else if (method=="maxprob_euc")  //doesn't guarantee convergance
        bpf = new QuasiOptPoseFinderNode(QuasiOptPoseFinder::MAXPROB_EUC);
    else if (method=="maxprob_obst") 
        bpf = new QuasiOptPoseFinderNode(QuasiOptPoseFinder::MAXPROB_OBST);
    else if (method=="cog_euq")      //doesn't guarantee convergance
        bpf = new QuasiOptPoseFinderNode(QuasiOptPoseFinder::COG_EUC);
    else if (method=="nearcog_euc")  //doesn't guarantee convergance
        bpf = new QuasiOptPoseFinderNode(QuasiOptPoseFinder::NEARCOG_EUC);
    else if (method=="nearcog_obst") 
        bpf = new QuasiOptPoseFinderNode(QuasiOptPoseFinder::NEARCOG_OBST);
    else if (method=="cog2lopt")
        bpf = new OptPoseFinderNode(OptPoseFinder::COG2LOPT);
    else if (method=="ramaxprob2lopt")
        bpf = new OptPoseFinderNode(OptPoseFinder::RAMAXPROB2LOPT);
    else if (method=="maxprob2lopt")     
        bpf = new OptPoseFinderNode(OptPoseFinder::MAXPROB2LOPT);
    else if (method=="gopt")
        bpf = new OptPoseFinderNode(OptPoseFinder::GOPT);
    else {
         ROS_ERROR("%s: wrong value for 'method' parameter ('%s'), will die now", getName().c_str(), method.c_str());
         return 1;
    }

    ros::spin();
    delete(bpf);
    return 0;
}
