/*
*/

#include <iostream>
#include <fstream>
#include <string>

#include <yaml-cpp/yaml.h>

#include <CompoundMap.h>
#include <CWave2.h>

#include "ros/ros.h"
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>

#include <lthmi_nav/StartExperiment.h>
#include <lthmi_nav/FloatMap.h>
#include <lthmi_nav/IntMap.h>
#include <lthmi_nav/Command.h>
#include <lthmi_nav/common.cpp>

using namespace cwave;
using namespace std;

namespace lthmi_nav {

typedef ros::Duration duration;
typedef ros::Time timestamp;
    
struct CourseRecord {
    string  commit;
    string  pos;
    string  div;
    bool    bag;
    string  mx;
    int     path;
    string  popt;
    string  map;
    string  rviz;
    double  resol;
    int     tries;
    double  vel;
    double  trobot;
    double  delay;
    double  phigh;
    double  plow;
    double  peps;
    double  ksafe;
    double  period;
    
    
    double length_ideal;
    double length_real;
    int n_decisions;
    int n_misdetected_decisions;
    int n_pois_inferred_correctly;
    duration dur_inference;
    duration dur_driving;
    duration dur_drinference;
    duration dur_calc_pdf;
    duration dur_calc_best_pose;
    duration dur_calc_map_div;
    //        overdrive_length = course_path_length_real/course_path_length_ideal
    //        overdrive_time = (time_pure_inference+time_drinference+time_pure_driving)/ideal_nav_time
    //        drinference duty (drinf time/nav time)
    //        driving duty (pure driving time/nav time)
    //        inference duty (pure inference time/nav time) # in my case should be close to 0
};
    
class MsgProcessor {
public:
    enum State { WAIT4POI, INFERENCE, DRIVING };
    State state;
    bool init_pose_defined_;
    bool first_run_;
    CourseRecord record_;
    string dir_;
    int try_;
    vector<Point> pois_;
    vector<Point> waypoints_;
    timestamp stamp_cmd_intended_;
    timestamp stamp_pdf_;
    timestamp stamp_pose_best_;
    timestamp stamp_map_divided_;
    timestamp stamp_pose_arrived_;
    timestamp stamp_pose_inferred_;
    int cmd_intended_;
    double resolution_;
    
    //CompoundMap cmap_;
    
    
    MsgProcessor() {
        state = WAIT4POI;
        first_run_ = true;
    }
    
    double calculateLengthToPOI() {
        return 1.0;
    }
    
    void writeTableRow (){
    }
    
    void desyncedMessage(const char* msg) {
        cerr << "Message from /" <<msg << " received when it shouldn't have been received\n";
        exit(1);
    }
    
    void paramCb(std_msgs::StringConstPtr msg) {
        YAML::Node prms = YAML::Load(msg->data.c_str());
        record_.commit  = prms["node_param_publisher"]["commit"].as<string>();
        record_.bag     = boost::lexical_cast<bool>(prms["node_param_publisher"]["bag"].as<string>());
        record_.mx      = prms["node_param_publisher"]["mx"].as<string>();
        record_.map     = prms["node_param_publisher"]["map"].as<string>();
        record_.path    = prms["node_param_publisher"]["path"].as<int>();
        record_.popt    = prms["node_param_publisher"]["popt"].as<string>();
        record_.rviz    = prms["node_param_publisher"]["rviz"].as<string>();
        record_.resol   = prms["experimentator"]["resol"].as<double>();
        record_.tries   = prms["experimentator"]["n_runs"].as<int>();
        record_.vel     = prms["robot_model"]["max_vel"].as<double>();
        record_.trobot  = prms["robot_model"]["pub_period"].as<double>();
        record_.delay   = prms["lthmi_model"]["delay"].as<double>();
        record_.phigh   = prms["inference_unit"]["thresh_high"].as<double>();
        record_.plow    = prms["inference_unit"]["thresh_low"].as<double>();
        record_.peps    = prms["inference_unit"]["eps"].as<double>();
        record_.pos     = prms["best_pose_finder"]["method"].as<string>();
        record_.ksafe   = prms["best_pose_finder"]["safety_coef"].as<double>();
        record_.period  = prms["best_pose_finder"]["period"].as<double>();
        record_.div     = prms["map_divider"]["method"].as<string>();
        //ROS_INFO(">>>>>>>>>>>>>>>>>>>> pos=%s, div=%s, commit=%s", record_.pos.c_str(), record_.div.c_str(), record_.commit.c_str());
    }
    
    void mapCb(IntMapConstPtr msg) { 
        ROS_INFO("got map=========================================");
        if (state==WAIT4POI) {
            resolution_ = msg->info.resolution;
            if (!first_run_)
                writeTableRow();
            first_run_ = false;
        }
    }
    
    void poseCurrentCb(geometry_msgs::PoseStampedConstPtr msg) {
        //ROS_INFO("got pose_current");
        if (!init_pose_defined_) {
            init_pose_defined_ = true;
            int x,y;
            updateVertex(msg->pose, x, y);
            pois_.push_back(Point(x,y));
            waypoints_.push_back(Point(x,y));
        }
    }
    
    void poseIntendedCb(geometry_msgs::PoseStampedConstPtr msg) {
        ROS_INFO("got pose_intended");
        stamp_cmd_intended_ = msg->header.stamp;
        if (state==WAIT4POI) {
            state = INFERENCE;
        }
    }
    
    void pdfCb(FloatMapConstPtr msg) {
        ROS_INFO("got pdf");
        if (state==INFERENCE) {
            stamp_pdf_ = msg->header.stamp;
            record_.dur_calc_pdf += stamp_pdf_-stamp_cmd_intended_;
        } else {
            desyncedMessage("pdf");
        }
    }
    
    void poseBestCb(geometry_msgs::PoseStampedConstPtr msg) {
        ROS_INFO("got pose_best");
        if (state==INFERENCE) {
            stamp_pose_best_ = msg->header.stamp;
            record_.dur_calc_best_pose += stamp_pose_best_-stamp_pdf_;
            //Point wp;
            //updateVertex(msg->pose, wp.x, wp.y);
        } else {
            desyncedMessage("pose_best");
        }
    }
    
    void mapDividedCb(IntMapConstPtr msg) {
        ROS_INFO("got map_divided");
        if (state==INFERENCE) {
            stamp_map_divided_ = msg->header.stamp;
            record_.dur_calc_map_div += stamp_map_divided_-stamp_pose_best_;
            record_.dur_driving += stamp_map_divided_-stamp_pose_best_;
        } else {
            desyncedMessage("map_divided");
        }
    }

    void poseArrivedCb(geometry_msgs::PoseStampedConstPtr msg) {
        ROS_INFO("got pose_arrived");
        if (state==INFERENCE || state==DRIVING) {
            stamp_pose_arrived_ = msg->header.stamp;
            Point wp;
            updateVertex(msg->pose, wp.x, wp.y);
            waypoints_.push_back(wp);
            if (state==DRIVING) {
                state = WAIT4POI;
                record_.length_real += calculateLengthToPOI();
                waypoints_.resize(0);
                waypoints_.push_back(wp);
                record_.dur_driving += stamp_pose_arrived_-stamp_pose_inferred_;
                ROS_INFO("=====arrived to POI ====");
            } else {
                record_.dur_drinference += stamp_pose_arrived_-stamp_map_divided_;
                ROS_INFO("== arrived to waypoint ==");
            }
        } else {
            desyncedMessage("pose_arrived");
        }
    }
    
    void cmdIntendedCb(CommandConstPtr msg) {
        ROS_INFO("got cmd_intended");
        if (state==INFERENCE) {
            cmd_intended_ = msg->cmd;
            record_.n_decisions++;
        } else {
            desyncedMessage("cmd_intended");
        }
    }
    
    void cmdDetectedCb(CommandConstPtr msg) {
        ROS_INFO("got cmd_detected");
        if (state==INFERENCE) {
            if (msg->cmd != cmd_intended_)
                record_.n_misdetected_decisions++;
        } else {
            desyncedMessage("cmd_detected");
        }
    }
    
    void poseInferredCb(geometry_msgs::PoseStampedConstPtr msg) {
        ROS_INFO("= got pose_inferred =");
        if (state==INFERENCE) {
            state = DRIVING;
            stamp_pose_inferred_ = msg->header.stamp;
            Point wp;
            updateVertex(msg->pose, wp.x, wp.y);
            if (wp.x==pois_.back().x && wp.y==pois_.back().y )
                record_.n_pois_inferred_correctly++;
        } else {
            desyncedMessage("pose_inferred");
        }
    }
    
    void updateVertex(const geometry_msgs::Pose& pose, int& x, int& y) {
        x = (int) round( pose.position.x / resolution_);
        y = (int) round( pose.position.y / resolution_);
    }
    
    
    void run() {
    }
};
}
