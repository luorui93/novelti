/*
       subs                                       pubs
                   +--------------------+
                   |                    | ---> /map_divided
/pose_optimal ---> |                    | 
         /pdf ---> |                    | ---> /debug_pose         |
                   |  node_map_divider  | ---> /debug_pose_border  |
                   |                    | ---> /debug_map_track    | debug
                   |                    | ---> /debug_map_dist     |
                   |                    | ---> /debug_stars        |
                   +--------------------+
                              ^
                              |
                        srv: start
                            req:  scene
                            resp: -
*/

#include <novelti/map_divider.h>
#include "map_divider_tile.cpp"
#include "map_divider_cwave.cpp"
#include "map_divider_vchess.cpp"

using namespace novelti;

class TileMapDividerNode : public TileMapDivider, public SynchronizableNode {
public:
    TileMapDividerNode(TileType type) : 
        TileMapDivider(type),
        SynchronizableNode()
        {}
    void start(novelti::StartExperiment::Request &req) {
        startExp(req);
    }

    void stop() {
        stopExp();
    }
}; 

class CWaveMapDividerNode : public CWaveMapDivider, public SynchronizableNode {
public:
    CWaveMapDividerNode(DivMethod method) :
        CWaveMapDivider(method),
        SynchronizableNode()
        {}
    void start(novelti::StartExperiment::Request &req) {
        startExp(req);
    }

    void stop() {
        stopExp();
    }
};

class VertChessMapDividerNode : public VertChessMapDivider, public SynchronizableNode {
public:
    void start(novelti::StartExperiment::Request &req) {
        startExp(req);
    }

    void stop() {
        stopExp();
    }
};
    

int main(int argc, char **argv) {
    ros::init(argc, argv, "map_divider");
    
    ros::NodeHandle n("~");
    std::string p = "htile";
    n.getParam("div/method", p);
    MapDivider* mdiv = nullptr;
    if      (p=="vtile")        mdiv = new TileMapDividerNode(TileMapDivider::VERT);
    else if (p=="htile")        mdiv = new TileMapDividerNode(TileMapDivider::HORIZ);
    else if (p=="altertile")    mdiv = new TileMapDividerNode(TileMapDivider::ALTER);
    else if (p=="equidist")     mdiv = new CWaveMapDividerNode(CWaveMapDivider::EQUIDIST);
    else if (p=="extremal")     mdiv = new CWaveMapDividerNode(CWaveMapDivider::EXTREMAL);
    else if (p=="extredist")    mdiv = new CWaveMapDividerNode(CWaveMapDivider::EXTREDIST);
    else if (p=="vchess")       mdiv = new VertChessMapDividerNode();
    else if (p=="nearcog_extremal") mdiv = new CWaveMapDividerNode(CWaveMapDivider::NEARCOG_EXTREMAL);
//     else if (p=="mixed1")      mdiv = Mixed1MapDividerNode();
//     else if (p=="mixed2")      mdiv = Mixed2MapDividerNode();
     else { 
         ROS_ERROR("%s: wrong value for 'method' parameter ('%s'), will die now", getName().c_str(), p.c_str());
         return 1;
     }
     ros::spin();
     delete(mdiv);

     return 0;
}
