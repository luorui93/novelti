#include <ncurses.h>  // getch
#include "ros/ros.h"
#include <string>
#include <low_throughput_hmi/map.h>

#include "low_throughput_hmi/IntMap.h"
#include <low_throughput_hmi/map_ros.cpp>

#include "../test/test_mazes.cpp"


#include <time.h>       /* time */

using namespace low_throughput_hmi_cost;
               
const int W=30, H=W, CX=int(W/2), CY=int(H/2);

map<const char*, vector<Wall> > mazes = generate_test_mazes(W, H);
 

int main(int argc, char **argv) {

    
    
    ros::init(argc, argv, "map_publisher");
    ros::NodeHandle n("~");

    //string maze_name;
    //n.getParam("mazeName", maze_name);
    
    low_throughput_hmi::IntMap map_msg;
        map_msg.info.width = W;
        map_msg.info.height = H;
        map_msg.info.resolution = 0.1;
        map_msg.data = vector<int>(W*H, -1);
        MapRos<low_throughput_hmi::IntMap, int>map(map_msg); 
    
    map.add_walls(mazes["pi"]);
    
    ros::Publisher pub =n.advertise<low_throughput_hmi::IntMap>("/map", 10);
    
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
