#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("chatter", 1000);
    ros::Rate loop_rate(1);
    ros::Duration(5).sleep();
    geometry_msgs::PoseStamped msg;
    while (ros::ok())  {
        msg.header.seq++;
        chatter_pub.publish(msg);
        ros::spinOnce();
        ROS_INFO("seq=%d", msg.header.seq);
        loop_rate.sleep();
    }
    return 0;
}