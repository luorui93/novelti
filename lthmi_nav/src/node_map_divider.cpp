//#define DEBUG_DIVIDER 1

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

using namespace lthmi_nav;

int main(int argc, char **argv) {
    ros::init(argc, argv, "map_divider");
    
    //ros::NodeHandle n("~");
    //std::string p;
    //n.getParam("division_policy", p);
    //VertTileMapDivider* mdiv = new VertTileMapDivider();
    VertTileMapDivider mdiv;
    mdiv.run();
    return 0;//mdiv.run1();
//    if      (p=="vert_tiles")  return VertTileMapDivider().run();
//     else if (p=="horiz_tiles") mdiv = Tile2by2MapDividerNode(false);
//     else if (p=="extremals")   mdiv = ExtremalsMapDividerNode();
//     else if (p=="equidists")   mdiv = EquiDist2MapDividerNode();
//     else if (p=="mixed1")      mdiv = Mixed1MapDividerNode();
//     else if (p=="mixed2")      mdiv = Mixed2MapDividerNode();
//     else { 
//         ROS_ERROR_NAMED(getNamespace(), "wrong value for 'division_policy' parameter, will die now"); 
//         return 1;
//     }
    
//     return mdiv.run();
}
