#include <novelti/novelti_shared_control.h>
#include "best_pose_finder_opt.cpp"
#include "map_divider_tile.cpp"
#include "map_divider_cwave.cpp"
#include "map_divider_vchess.cpp"
#include <novelti/best_pose_finder_quasi_opt.h>

using namespace novelti;



NoveltiSharedControl::NoveltiSharedControl (std::string divMethod, std::string posMethod):
    SynchronizableNode(),
    divMethod(divMethod),
    posMethod(posMethod)
{  
    //best_pose_finder initialization
    if      (posMethod=="no_move")
        bpf = new BestPoseFinder("pos");
    else if (posMethod=="ra_maxprob")   
        bpf = new QuasiOptPoseFinder(QuasiOptPoseFinder::RA_MAXPROB,"pos");
    else if (posMethod=="maxprob_euc")  //doesn't guarantee convergance
        bpf = new QuasiOptPoseFinder(QuasiOptPoseFinder::MAXPROB_EUC,"pos");
    else if (posMethod=="maxprob_obst") 
        bpf = new QuasiOptPoseFinder(QuasiOptPoseFinder::MAXPROB_OBST,"pos");
    else if (posMethod=="cog_euq")      //doesn't guarantee convergance
        bpf = new QuasiOptPoseFinder(QuasiOptPoseFinder::COG_EUC,"pos");
    else if (posMethod=="nearcog_euc")  //doesn't guarantee convergance
        bpf = new QuasiOptPoseFinder(QuasiOptPoseFinder::NEARCOG_EUC,"pos");
    else if (posMethod=="nearcog_obst") 
        bpf = new QuasiOptPoseFinder(QuasiOptPoseFinder::NEARCOG_OBST,"pos");
    else if (posMethod=="cog2lopt")
        bpf = new OptPoseFinder(OptPoseFinder::COG2LOPT,"pos");
    else if (posMethod=="ramaxprob2lopt")
        bpf = new OptPoseFinder(OptPoseFinder::RAMAXPROB2LOPT,"pos");
    else if (posMethod=="maxprob2lopt")     
        bpf = new OptPoseFinder(OptPoseFinder::MAXPROB2LOPT,"pos");
    else if (posMethod=="gopt")
        bpf = new OptPoseFinder(OptPoseFinder::GOPT,"pos");
    else {
         ROS_ERROR("%s: wrong value for 'posMethod' parameter ('%s'), will die now", getName().c_str(), posMethod.c_str());
    }

    //map_divider initialization
    if      (divMethod=="vtile")        mdiv = new TileMapDivider(TileMapDivider::VERT,"div");
    else if (divMethod=="htile")        mdiv = new TileMapDivider(TileMapDivider::HORIZ,"div");
    else if (divMethod=="altertile")    mdiv = new TileMapDivider(TileMapDivider::ALTER,"div");
    else if (divMethod=="equidist")     mdiv = new CWaveMapDivider(CWaveMapDivider::EQUIDIST,"div");
    else if (divMethod=="extremal")     mdiv = new CWaveMapDivider(CWaveMapDivider::EXTREMAL,"div");
    else if (divMethod=="extredist")    mdiv = new CWaveMapDivider(CWaveMapDivider::EXTREDIST,"div");
    else if (divMethod=="vchess")       mdiv = new VertChessMapDivider("div");
    else if (divMethod=="nearcog_extremal") mdiv = new CWaveMapDivider(CWaveMapDivider::NEARCOG_EXTREMAL,"div");
//     else if (p=="mixed1")      mdiv = Mixed1MapDividerNode();
//     else if (p=="mixed2")      mdiv = Mixed2MapDividerNode();
     else { 
         ROS_ERROR("%s: wrong value for 'divMethod' parameter ('%s'), will die now", getName().c_str(), divMethod.c_str());
     }

     iu = new InferenceUnit("inf");
}

FloatMapConstPtr floatMapToPtr (FloatMap map) {
    FloatMapConstPtr ptr(new FloatMap(map));
    return ptr;
}

IntMapConstPtr intMapToPtr (IntMap map) {
    IntMapConstPtr ptr(new IntMap(map));
    return ptr;
}

geometry_msgs::PoseStampedConstPtr poseStampedToPtr (geometry_msgs::PoseStamped pose) {
    geometry_msgs::PoseStampedConstPtr ptr(new geometry_msgs::PoseStamped(pose));
    return ptr;
}

void NoveltiSharedControl::start(novelti::StartExperiment::Request& req) {
    ROS_INFO("Starting....");
    setInferenceStarted(false);
    iu->startExp(req);
    ROS_INFO("Inference Unit Started...");
    bpf->startExp(req);
    ROS_INFO("Best Pose Finder Started...");
    mdiv->startExp(req);
    ROS_INFO("Map Divider Started...");
    sub_cmd = node.subscribe("/cmd_detected", 1, &NoveltiSharedControl::cmdCallback, this);
    srv_new_goal = node.advertiseService("new_goal", &NoveltiSharedControl::srvNewGoal, this);
}

void NoveltiSharedControl::stop() {
    ROS_INFO("STOPPING...");
    iu->stopExp();
    bpf->stopExp();
    mdiv->stopExp();
}

bool NoveltiSharedControl::srvNewGoal(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp) {
    ROS_INFO("new_goal advertised");
    iu->srvNewGoal(req, resp);
    //udpate pose_beset
    bpf->pdfCallback(floatMapToPtr(iu->pdf));
    //update map_divided
    mdiv->noveltiMapCallback(floatMapToPtr(iu->pdf),poseStampedToPtr(bpf->pose_best));
    setInferenceStarted(true);
    return true;
}

void NoveltiSharedControl::cmdCallback(CommandConstPtr cmd) {
    if (isInferenceStarted()) {
        iu->noveltiInfCallback(intMapToPtr(mdiv->map_divided),cmd);
        if (iu->state == InferenceUnit::INFERRED) {
            setInferenceStarted(false);
            ROS_INFO("Desired Pose Inferred...");
        }
        //posecurcallback?
        bpf->pdfCallback(floatMapToPtr(iu->pdf));
        mdiv->noveltiMapCallback(floatMapToPtr(iu->pdf),poseStampedToPtr(bpf->pose_best));
    }
}

//set a flag to ensure map_divided is initialized
void NoveltiSharedControl::setInferenceStarted(bool value) {
    inference_started_lock.lock();
    inference_started = value;
    inference_started_lock.unlock();
}

bool NoveltiSharedControl::isInferenceStarted() {
    bool value;
    inference_started_lock.lock();
    value = inference_started;
    inference_started_lock.unlock();
    return value;
}

