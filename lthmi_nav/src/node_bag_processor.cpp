#include <ros/ros.h>
#include "msg_processor.cpp"

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

using namespace lthmi_nav;

class BagProcessor: public MsgProcessor {
public:
    rosbag::Bag bag_;
    std::vector<std::string> topics_;
    
    BagProcessor(string& bag_path, ostream& out) :
        MsgProcessor(out)
    {
        bag_.open(bag_path.c_str(), rosbag::bagmode::Read);
        topics_.push_back("/parameters");
        topics_.push_back("/map");
        topics_.push_back("/pose_current");
        topics_.push_back("/pose_intended");
        topics_.push_back("/pdf");
        topics_.push_back("/pose_best");
        topics_.push_back("/map_divided");
        topics_.push_back("/pose_arrived");
        topics_.push_back("/pose_current");
        topics_.push_back("/cmd_intended");
        topics_.push_back("/cmd_detected");
        topics_.push_back("/pose_inferred");
    }
    
    void run() {
        rosbag::View view(bag_, rosbag::TopicQuery(topics_));
        foreach(rosbag::MessageInstance const m, view) {
            const string topic = m.getTopic();
            if (topic == "/parameters")         {   paramCb(m.instantiate<std_msgs::String>()); }
            else if (topic == "/map")           {   mapCb(m.instantiate<IntMap>()); } 
            else if (topic == "/pose_current")  {   poseCurrentCb(m.instantiate<geometry_msgs::PoseStamped>()); }
            else if (topic == "/pose_intended") {   poseIntendedCb(m.instantiate<geometry_msgs::PoseStamped>()); }
            else if (topic == "/pdf")           {   pdfCb(m.instantiate<FloatMap>()); }
            else if (topic == "/pose_best")     {   poseBestCb(m.instantiate<geometry_msgs::PoseStamped>()); }
            else if (topic == "/map_divided")   {   mapDividedCb(m.instantiate<IntMap>()); }
            else if (topic == "/pose_arrived")  {   poseArrivedCb(m.instantiate<geometry_msgs::PoseStamped>()); }
            else if (topic == "/pose_current")  {   poseCurrentCb(m.instantiate<geometry_msgs::PoseStamped>()); }
            else if (topic == "/cmd_intended")  {   cmdIntendedCb(m.instantiate<Command>()); }
            else if (topic == "/cmd_detected")  {   cmdDetectedCb(m.instantiate<Command>()); }
            else if (topic == "/pose_inferred") {   poseInferredCb(m.instantiate<geometry_msgs::PoseStamped>()); }
        }
        finish();
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "bag_processor");
    ros::NodeHandle node("~");
    string bag_path, stats_path;
    node.getParam("bag_path", bag_path);
    node.getParam("stats_path", stats_path);
    if (bag_path.size()<=0) {
        ROS_ERROR("ROS parameter bag_path is required");
        return 1;
    }
    
    if (stats_path.size()>0) {
        ofstream stats_stream;
        stats_stream.open(stats_path);
        BagProcessor pr(bag_path, stats_stream);
        pr.run();
        stats_stream.close();
    } else {
        BagProcessor pr(bag_path, std::cout);
        pr.run();
    }
    return 0;
}
