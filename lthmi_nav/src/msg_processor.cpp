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

class Record {
public:
    static const char s = '\t';
    typedef char* doc;
};



class CourseParams : public Record {
public:
    timestamp   run;    //static doc doc_run      = "Time ...";
    timestamp   start;  //static doc doc_start    = "Timestamp from the /pose_intended message in each run";
    string      commit; //static doc doc_commit   = "git commit id of the code that was used to create the rosbag";

    string      map;    //static doc doc_map      = "Name of the map (without .map extension)";
    int         path;   //static doc doc_path     = "Integer ID of the path from YOUR_MAP.paths file";
    double      resol;  //static doc doc_resol    = "Cell width on the map (in meters)";
    int         tries;  //static doc doc_tries    = "Number of tries this experiment was run";

    string      mx;     //static doc doc_mx       = "Name of the HMI matrix";
    double      period; //static doc doc_period   = "LT HMI update period (in seconds)";
    double      vel;    //static doc doc_vel      = "Robot velocity (in meters/sec)";
    double      trobot; //static doc doc_trobot   = "Period at which robot_model publishes its current pose";

    double      phigh;  //static doc doc_phigh    = "When probability that a given vertex is an intended destination reaches this value, the vertex is consedered an inferred destination (no pdf is published)";
    double      plow;   //static doc doc_plow     = "After a previously inferred destination is reached, and new_goal service call is made to inference_unit, it will wait until the highest vertex probability decreases to this value, before it starts checking for phigh";
    double      peps;   //static doc doc_peps     = "Whenever a vertex has a probaility less than this value, its increased to this value, and PDF is normalized";

    string      pos;    //static doc doc_pos      = "Name of the method name that was used to find best pose";
    double      ksafe;  //static doc doc_ksafe    = "When reachability area is calculated, robot velocity is multiplied by this value to ensure robot always reach /pose_best";
    
    string      div;    //static doc doc_div      = "Name of the method that was used to divide map";
    string      popt;   //static doc doc_popt     = "Name of the array with optimal probabilities (array values should correspond to the interface matrix)";
    
    bool        bag;    //static doc doc_bag      = "Boolean: was bag file created pr not";
    string      rviz;   //static doc doc_rviz     = "Type of the rviz config that was used when running experiment (none, static or autocam)";
    double      delay;  //static doc doc_delay    = "HMI delay (should be equal to period)";
    
    void headerOut(ostream& out) {
        out << 
            setw(14)<<"run" << setw(14)<<"start" << setw(10)<<"commit" << 
            setw(16)<<"map" << setw(7)<<"path"   << setw(7)<<"resol"   << setw(7)<<"tries"<<
            setw( 8)<<"mx"  << setw(8)<<"period" << setw(8)<<"vel"     << setw(10)<<"trobot"<< 
            setw( 8)<<"phigh"<< setw(8)<<"plow"   << setw(10)<<"peps"   <<
            setw(15)<<"pos" << setw(8)<<"ksafe"  <<
            setw(15)<<"div" << setw(15)<<"popt"  <<
            setw( 6)<<"bag" << setw(10)<<"rviz"  << setw(8)<<"delay";
    }
};

ostream& operator<<(ostream& out, const CourseParams& r) {
    char s = CourseParams::s;
    return cout << 
        setw(14)<<r.run  << setw(14)<<r.start  << setw(10)<<r.commit << 
        setw(16)<<r.map  << setw(7)<<r.path    << setw(7)<<r.resol   << setw(7)<<r.tries <<
        setw(8)<<r.mx    << setw(8)<<r.period  << setw(8)<<r.vel     << setw(10)<<r.trobot << 
        setw(8)<<r.phigh << setw(8)<<r.plow    << setw(10)<<r.peps   <<
        setw(15)<<r.pos  << setw(8)<<r.ksafe   <<
        setw(15)<<r.div  << setw(15)<<r.popt   <<
        setw(6)<<r.bag   << setw(10)<<r.rviz   << setw(8)<<r.delay;
}




class CourseStats : public Record {
public:
    double l_ideal; //"Length of the shortest path connecting all POIs in the order they are given"
    double l_real;  //"Length of the actual path made by the robot to visit all POIs"
    int dcs_total;  //"Total number of decisions the user had to make while proceeding thorugh this course"
    int dcs_wrong;  //"Number of decisions that have been incorrectly detected"
    int poi_total;  //"Total number of POIs visited in this course"
    int poi_wrong;  //"Number of POIs that were inferred incorrectly"
    duration t_inf;     //"The accumulated amount of time spent purely on inference"
    duration t_drive;   //"The accumulated amount of time spent purely on driving"
    duration t_drinf;   //"The accumulated amount of time spent on simulteneous driving and inference"
    duration t_pdf;     //"The accumulated amount of time spent calculating PDFs"
    duration t_pos;     //"The accumulated amount of time spent on search for bet pose"
    duration t_div;     //"The accumulated amount of time spent on map division"
    //        overdrive_length = course_path_l_real/course_path_length_ideal"
    //        overdrive_time = (time_pure_inference+time_drinference+time_pure_driving)/ideal_nav_time
    //        drinference duty (drinf time/nav time)
    //        driving duty (pure driving time/nav time)
    //        inference duty (pure inference time/nav time) # in my case should be close to 0
    
    void headerOut(ostream& out) {
        out << 
        setw(12)<<"l_ideal"     <<setw(12)<<"l_real"    <<setw( 9)<<"dcs_total" <<setw(9)<<"dcs_wrong"   <<
        setw( 9)<<"poi_total"   <<setw( 9)<<"poi_wrong" <<
        setw(16)<<"t_inf"       <<setw(16)<<"t_drive"   <<setw(16)<<"t_drinf"    <<
        setw(16)<<"t_pdf"       <<setw(16)<<"t_pos"     <<setw(16)<<"t_div";
    }
};
    

ostream& operator<<(ostream& out, const CourseStats& v) {
    return out << 
            setw(12)<<v.l_ideal     <<setw(12)<<v.l_real    <<setw( 9)<<v.dcs_total <<setw(9)<<v.dcs_wrong   <<
            setw( 9)<<v.poi_total   <<setw( 9)<<v.poi_wrong <<
            setw(16)<<v.t_inf       <<setw(16)<<v.t_drive   <<setw(16)<<v.t_drinf    <<
            setw(16)<<v.t_pdf       <<setw(16)<<v.t_pos     <<setw(16)<<v.t_div;    
}

class MsgProcessor {
public:
    enum State { WAIT4POI, INFERENCE, DRIVING };
    State state;
    bool init_pose_defined_;
    bool first_run_;
    CourseParams prms_;
    CourseStats  stats_;
    
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
        writeTableHeader();
    }
    
    double calculateLengthToPOI() {
        return 1.0;
    }
    
    void writeTableHeader () {
        prms_.headerOut(cout);
        cout << "    ";
        stats_.headerOut(cout);
        cout << endl;
    }

    void resetStats() {
        stats_.l_ideal   = 0.0;
        stats_.l_real    = 0.0;
        stats_.dcs_total = 0;
        stats_.dcs_wrong = 0;
        stats_.poi_total = 0;
        stats_.poi_wrong = 0;
        stats_.t_inf     = ros::Duration(0);
        stats_.t_drive   = ros::Duration(0);
        stats_.t_drinf   = ros::Duration(0);
        stats_.t_pdf     = ros::Duration(0);
        stats_.t_pos     = ros::Duration(0);
        stats_.t_div     = ros::Duration(0);
    }
    
    void writeTableRow () {
        cout << prms_ << Record::s << 
            stats_ << endl;
        resetStats();
    }
    
    void desyncedMessage(const char* msg) {
        cerr << "Message from /" <<msg << " received when it shouldn't have been received\n";
        exit(1);
    }
    
    void paramCb(std_msgs::StringConstPtr msg) {
        YAML::Node prms = YAML::Load(msg->data.c_str());
        prms_.commit  = prms["node_param_publisher"]["commit"].as<string>().substr(0,7); 
        prms_.bag     = boost::lexical_cast<bool>(prms["node_param_publisher"]["bag"].as<string>());
        prms_.mx      = prms["node_param_publisher"]["mx"].as<string>();
        prms_.map     = prms["node_param_publisher"]["map"].as<string>();
        prms_.path    = prms["node_param_publisher"]["path"].as<int>();
        prms_.popt    = prms["node_param_publisher"]["popt"].as<string>();
        prms_.rviz    = prms["node_param_publisher"]["rviz"].as<string>();
        prms_.resol   = prms["experimentator"]["resol"].as<double>();
        prms_.tries   = prms["experimentator"]["n_runs"].as<int>();
        prms_.vel     = prms["robot_model"]["max_vel"].as<double>();
        prms_.trobot  = prms["robot_model"]["pub_period"].as<double>();
        prms_.delay   = prms["lthmi_model"]["delay"].as<double>();
        prms_.phigh   = prms["inference_unit"]["thresh_high"].as<double>();
        prms_.plow    = prms["inference_unit"]["thresh_low"].as<double>();
        prms_.peps    = prms["inference_unit"]["eps"].as<double>();
        prms_.pos     = prms["best_pose_finder"]["method"].as<string>();
        prms_.ksafe   = prms["best_pose_finder"]["safety_coef"].as<double>();
        prms_.period  = prms["best_pose_finder"]["period"].as<double>();
        prms_.div     = prms["map_divider"]["method"].as<string>();
        //ROS_INFO(">>>>>>>>>>>>>>>>>>>> pos=%s, div=%s, commit=%s", prms_.pos.c_str(), prms_.div.c_str(), prms_.commit.c_str());
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
            stats_.t_pdf += stamp_pdf_-stamp_cmd_intended_;
        } else {
            desyncedMessage("pdf");
        }
    }
    
    void poseBestCb(geometry_msgs::PoseStampedConstPtr msg) {
        ROS_INFO("got pose_best");
        if (state==INFERENCE) {
            stamp_pose_best_ = msg->header.stamp;
            stats_.t_pos += stamp_pose_best_-stamp_pdf_;
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
            stats_.t_div += stamp_map_divided_-stamp_pose_best_;
            stats_.t_drive += stamp_map_divided_-stamp_pose_best_;
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
                stats_.l_real += calculateLengthToPOI();
                waypoints_.resize(0);
                waypoints_.push_back(wp);
                stats_.t_drive += stamp_pose_arrived_-stamp_pose_inferred_;
                ROS_INFO("=====arrived to POI ====");
            } else {
                stats_.t_drive += stamp_pose_arrived_-stamp_map_divided_;
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
            stats_.dcs_total++;
        } else {
            desyncedMessage("cmd_intended");
        }
    }
    
    void cmdDetectedCb(CommandConstPtr msg) {
        ROS_INFO("got cmd_detected");
        if (state==INFERENCE) {
            if (msg->cmd != cmd_intended_)
                stats_.dcs_wrong++;
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
            if (wp.x!=pois_.back().x || wp.y!=pois_.back().y )
                stats_.poi_wrong++;
        } else {
            desyncedMessage("pose_inferred");
        }
    }
    
    void finish() {
        writeTableRow();
    }
    void updateVertex(const geometry_msgs::Pose& pose, int& x, int& y) {
        x = (int) round( pose.position.x / resolution_);
        y = (int) round( pose.position.y / resolution_);
    }
    
    
    void run() {
    }
};
}
