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

#include <lthmi_nav/map_divider.h>
#include "map_divider_tile.cpp"
#include "map_divider_cwave.cpp"
//#include "map_divider_equidist.cpp"
//#include "map_divider_extremal.cpp"
//#include "map_divider_extremal_sort.cpp"
#include "map_divider_vchess.cpp"

using namespace lthmi_nav;

int main(int argc, char **argv) {
    ros::init(argc, argv, "map_divider");
    
    ros::NodeHandle n("~");
    std::string p = "htile";
    n.getParam("method", p);
    MapDivider* mdiv = nullptr;
    if      (p=="vtile")        mdiv = new TileMapDivider(TileMapDivider::VERT);
    else if (p=="htile")        mdiv = new TileMapDivider(TileMapDivider::HORIZ);
    else if (p=="altertile")    mdiv = new TileMapDivider(TileMapDivider::ALTER);
    else if (p=="equidist")     mdiv = new CWaveMapDivider(CWaveMapDivider::EQUIDIST);
    else if (p=="extremal")     mdiv = new CWaveMapDivider(CWaveMapDivider::EXTREMAL);
    else if (p=="extredist")    mdiv = new CWaveMapDivider(CWaveMapDivider::EXTREDIST);
    else if (p=="vchess")       mdiv = new VertChessMapDivider();
//     else if (p=="mixed1")      mdiv = Mixed1MapDividerNode();
//     else if (p=="mixed2")      mdiv = Mixed2MapDividerNode();
     else { 
         ROS_ERROR("%s: wrong value for 'method' parameter ('%s'), will die now", getName().c_str(), p.c_str());
         return 1;
     }
     mdiv->run();
     delete(mdiv);
}
