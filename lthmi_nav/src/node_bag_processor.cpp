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
    
    BagProcessor(string& bag_path) {
        bag_.open(bag_path.c_str(), rosbag::bagmode::Read);
        topics_.push_back(std::string("/parameters"));
    }
    
    void run() {
        rosbag::View view(bag_, rosbag::TopicQuery(topics_));
        foreach(rosbag::MessageInstance const m, view) {
            if (m.getTopic() == "/parameters" || m.getTopic() == "parameters") {
                std_msgs::StringConstPtr prm_str = m.instantiate<std_msgs::String>();
                paramCb(prm_str);
 /*               YAML::Node prms = YAML::Load(prm_str->data.c_str());
                //ROS_INFO(">>>>>>>>>>>>>>>>>>>> %s", method.c_str());
                if (prms["map_divider"]["method"]) {
                    string method = prms["map_divider"]["method"].as<string>().c_str();
                    ROS_INFO(">>>>>>>>>>>>>>>>>>>> %s", method.c_str());
                }
                
                //ROS_INFO("%s",prm_str->data.c_str());*/
            }
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "bag_processor");
    ros::NodeHandle node("~");
    string bag_path;
    node.getParam("bag_path", bag_path);
    if (bag_path.size()<=0) {
        ROS_ERROR("ROS parameter bag_path is required");
        return 1;
    }
    BagProcessor pr(bag_path);
    pr.run();
    return 0;
}
