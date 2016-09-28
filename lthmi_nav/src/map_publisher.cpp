#include <ncurses.h>  // getch
#include "ros/ros.h"
#include <string>
#include <lthmi_nav/map.h>

#include "lthmi_nav/IntMap.h"
#include <lthmi_nav/map_ros.cpp>

#include "../test/test_mazes.cpp"


#include <time.h>       /* time */

using namespace lthmi_nav;
               
const int W=30, H=W, CX=int(W/2), CY=int(H/2);

map<const char*, vector<Wall> > mazes = generate_test_mazes(W, H);
 

int main(int argc, char **argv) {

    
    
    ros::init(argc, argv, "map_publisher");
    ros::NodeHandle n("~");

    //string maze_name;
    //n.getParam("mazeName", maze_name);
    
    lthmi_nav::IntMap map_msg;
        map_msg.info.width = W;
        map_msg.info.height = H;
        map_msg.info.resolution = 0.1;
        map_msg.data = vector<int>(W*H, -1);
        MapRos<lthmi_nav::IntMap, int>map(map_msg); 
    
    map.add_walls(mazes["pi"]);
    
    ros::Publisher pub =n.advertise<lthmi_nav::IntMap>("/map", 10);
    
    ros::Rate loop_rate(0.002);
    
    sleep(12);
    
    while (ros::ok()) {
        pub.publish(*(map.get_msg()));
        ROS_INFO("map_publisher: published map");
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

}
