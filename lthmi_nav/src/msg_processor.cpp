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
    timestamp   run;    //static doc doc_run      = "Timestamp of the first /map message (can identify the bag)";
    int         tries;  //static doc doc_tries    = "Number of tries this experiment was run";
    int         tryidx; //static doc doc_tryidx   = "Try's index";
    timestamp   start;  //static doc doc_start    = "Timestamp from the /pose_intended message in each run";
    string      commit; //static doc doc_commit   = "git commit id of the code that was used to create the rosbag";

    string      map;    //static doc doc_map      = "Name of the map (without .map extension)";
    int         path;   //static doc doc_path     = "Integer ID of the path from YOUR_MAP.paths file";
    double      resol;  //static doc doc_resol    = "Cell width on the map (in meters)";

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
        out << boost::format(
        "%22s  %5s  %3s  %22s  %10s  %16s  %5s  %6s  %8s  %6s  %7s  %7s  %5s  %5s  %8s  %16s  %5s  %12s  %10s  %3s  %10s  %6s") %
        "run" % "tries" % "try" % "start" % "commit" % "map" % "path"  % "resol" % "mx" % "period" % "vel" % "trobot" % "phigh" % "plow" % "peps" % "pos" % "ksafe" % "div" % "popt" % "bag" % "rviz" % "delay";
    }
};

ostream& operator<<(ostream& out, const CourseParams& r) {
    char s = CourseParams::s;
    return out << boost::format(
      // run   trs  try  strt  cmmt  map   pth  resol  mx   perd   vel    trobo  phigh  plow   eps    pos   ksafe  div   popt  bag  rviz  delay;
        "%22f  %5d  %3d  %22f  %10s  %16s  %5d  %6.3f  %8s  %6.4f  %7.4f  %7.5f  %5.3f  %5.3f  %8.2e  %16s  %5.3f  %12s  %10s  %3d  %10s  %6.4f") %
        r.run % r.tries % r.tryidx % r.start % r.commit % r.map  % r.path % r.resol % r.mx % r.period % r.vel % r.trobot % r.phigh % r.plow % r.peps % r.pos % r.ksafe % r.div % r.popt % r.bag % r.rviz % r.delay;
}




class CourseStats : public Record {
public:
    double l_ideal; //"Length of the shortest path connecting all POIs in the order they are given"
    double l_real;  //"Length of the actual path made by the robot to visit all POIs"
    int dcs_total;  //"Total number of decisions the user had to make while proceeding thorugh this course"
    int dcs_wrong;  //"Number of decisions that have been incorrectly detected"
    int poi_total;  //"Total number of POIs visited in this course"
    int poi_wrong;  //"Number of POIs that were inferred incorrectly"
    int waypts;    //"Total number of waypoints (includes POIs as well)"
    duration t_inf;     //"The accumulated amount of time spent purely on inference"
    duration t_drive;   //"The accumulated amount of time spent purely on driving"
    duration t_drinf;   //"The accumulated amount of time spent on simulteneous driving and inference"
    duration t_pdf;     //"The accumulated amount of time spent calculating PDFs"
    duration t_pos;     //"The accumulated amount of time spent on search for bet pose"
    duration t_div;     //"The accumulated amount of time spent on map division"
    duration t_nav;     //"Total navigation time: t_nav = t_pdf + t_pos + t_drive + t_drinf + t_inf (t_nav = arrived_t(DRIVING)-pose_intended_t)"
    //        overdrive_length = course_path_l_real/course_path_length_ideal"
    //        overdrive_time = (time_pure_inference+time_drinference+time_pure_driving)/ideal_nav_time
    //        drinference duty (drinf time/nav time)
    //        driving duty (pure driving time/nav time)
    //        inference duty (pure inference time/nav time) # in my case should be close to 0
    
    void headerOut(ostream& out) {
        out << boost::format(
            "%12s  %12s  %9s  %9s  %9s  %9s  %6d  %18s  %18s  %18s  %18s  %18s  %18s  %18s") %
            "l_ideal" % "l_real" % "dcs_total" % "dcs_wrong" % "poi_total" % "poi_wrong" % "waypts" % "t_nav" % "t_inf" % "t_drive" % "t_drinf" % "t_pdf" % "t_pos" % "t_div";
    }
};
    

ostream& operator<<(ostream& out, const CourseStats& v) {
    return out << boost::format(
      // l_idea  l_real  d_t  d_w  p_t  p_w  wpts t_inf   t_driv  t_drnf  t_pdf   t_pos   t_div
        "%12.3f  %12.3f  %9d  %9d  %9d  %9d  %6d  %18.6f  %18.6f  %18.6f  %18.6f  %18.6f  %18.6f  %18.6f") %
        v.l_ideal % v.l_real % v.dcs_total % v.dcs_wrong % v.poi_total  % v.poi_wrong % v.waypts % v.t_nav % v.t_inf % v.t_drive % v.t_drinf % v.t_pdf % v.t_pos % v.t_div;
}

class MsgProcessor {
public:
    enum State { WAIT4POI, INFERENCE, DRIVING };
    State state;
    
    ostream& sout;
    
    bool init_pose_defined_;
    bool first_run_;
    CourseParams prms_;
    CourseStats  stats_;
    
    string dir_;
    int try_;
    vector<Point> pois_;
    vector<Point> waypoints_;
    timestamp stamp_pose_intended_;
    timestamp stamp_cmd_detected_;
    timestamp stamp_cmd_intended_;
    timestamp stamp_pdf_;
    timestamp stamp_pose_best_;
    timestamp stamp_map_divided_;
    timestamp stamp_pose_arrived_;
    timestamp stamp_pose_inferred_;
    int cmd_intended_;
    double resolution_;
    
    CompoundMap cmap_;
    
    
    MsgProcessor(ostream& outs) :
        sout(outs)
    {
        state = WAIT4POI;
        prms_.run = ros::Time(0);
        prms_.tryidx = 0;
        pois_.push_back(Point());
        waypoints_.push_back(Point());
        first_run_ = true;
        resetStats();
        writeTableHeader();
    }

    void readMap(IntMapConstPtr msg) { 
        new (&cmap_) CompoundMap(msg->info.width, msg->info.height);
        //ROS_INFO("w=%d, h=%d", msg->info.width, msg->info.height);
        for (int x=0; x<msg->info.width; x++)
            for (int y=0; y<msg->info.height; y++)
                if (msg->data[x + y*msg->info.width]==255)
                    cmap_.setPixel(x,y, FREED); //free
    }
    
    double calculateDist(Point& p1, Point& p2) {
        CWave2 cw(cmap_);
        CWave2Processor dummy;
        cw.setProcessor(&dummy);
        cw.calc(p1);
        double d = resolution_*cmap_.getExactDist(p2.x, p2.y);
        //double d =0.5*cmap_.getPoint(p2.x, p2.y); 
        //int dd = cmap_.getPoint(p2.x, p2.y);
        cmap_.clearDist();
        ROS_DEBUG("Calculated distance from (%d,%d) to (%d,%d) -> %f", p1.x, p1.y, p2.x, p2.y, d);
        return d;
    }
    
    void printPath(vector<Point> path) {
        for (const auto& pt: path)
            ROS_DEBUG( "(%d,%d) ", pt.x,pt.y);
        ROS_DEBUG("\n");
    }
    
    double calculatePathLength(vector<Point> path) {
        double d = 0.0;
        for (int k=1; k<path.size(); k++) {
            d += calculateDist(path[k-1], path[k]);
        }
        return d;
    }
    
    void writeTableHeader () {
        prms_.headerOut(sout);
        sout << "  ";
        stats_.headerOut(sout);
        sout << endl;
    }
    
    void writeTableRow () {
        prms_.tryidx++;
        stats_.l_ideal = calculatePathLength(pois_);
        sout << prms_ << "  " <<  stats_ << endl;
        //ROS_DEBUG("POIs: (%d,%d), (%d,%d), (%d,%d)", pois_[0].x, pois_[0].y, pois_[1].x, pois_[1].y, pois_[2].x, pois_[2].y);
        resetStats();
    }
    
    void resetStats() {
        init_pose_defined_ = false;
        pois_.resize(1);
        waypoints_.resize(1);
        stamp_cmd_detected_ = ros::Time(0);
        
        stats_.l_ideal   = 0.0;
        stats_.l_real    = 0.0;
        stats_.dcs_total = 0;
        stats_.dcs_wrong = 0;
        stats_.poi_total = 0;
        stats_.poi_wrong = 0;
        stats_.waypts    = 0;
        stats_.t_inf     = ros::Duration(0);
        stats_.t_drive   = ros::Duration(0);
        stats_.t_drinf   = ros::Duration(0);
        stats_.t_pdf     = ros::Duration(0);
        stats_.t_pos     = ros::Duration(0);
        stats_.t_div     = ros::Duration(0);
        stats_.t_nav     = ros::Duration(0);
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
        ROS_DEBUG("got map =========================================");
        readMap(msg);
        if (prms_.run==ros::Time(0))
            prms_.run = msg->header.stamp;
        if (state==WAIT4POI) {
            resolution_ = msg->info.resolution;
            if (!first_run_) {
                writeTableRow();
            }
            first_run_ = false;
        }
    }
    
    void poseCurrentCb(geometry_msgs::PoseStampedConstPtr msg) {
        //ROS_DEBUG("got pose_current: (%f,%f)", msg->pose.position.x, msg->pose.position.y);
        if (!init_pose_defined_) {
            init_pose_defined_ = true;
            Point pt;
            updateVertex(msg->pose, pt.x, pt.y);
            pois_[0] = pt;
            ROS_DEBUG("Added POI=(%d,%d)", pt.x, pt.y);
            waypoints_[0] = pt;
        }
    }
    
    void poseIntendedCb(geometry_msgs::PoseStampedConstPtr msg) {
        ROS_DEBUG("got pose_intended");
        stamp_pose_intended_ = msg->header.stamp;
        if (state==WAIT4POI) {
            prms_.start = stamp_pose_intended_;
            Point pt;
            updateVertex(msg->pose, pt.x, pt.y);
            pois_.push_back(pt);
            ROS_DEBUG("Added POI=(%d,%d)", pt.x, pt.y);
            stats_.poi_total++;
            state = INFERENCE;
        }
    }
    
    void pdfCb(FloatMapConstPtr msg) {
        ROS_DEBUG("got pdf");
        if (state==INFERENCE) {
            stamp_pdf_ = msg->header.stamp;
            ros::Duration dt;
            //ROS_INFO(">--- cmd_det=%f, 0t=%f", stamp_cmd_detected_.toSec(),ros::Time(0).toSec());
            if (stamp_cmd_detected_==ros::Time(0)) {
                dt = stamp_pdf_-stamp_pose_intended_;
            } else {
                dt = stamp_pdf_-stamp_cmd_detected_;
            }
            stats_.t_pdf += dt;
            //ROS_INFO(">>>>>> added to t_pdf=%f : (%d , %d)=%f - (%d , %d)=%f", dt.toSec(), stamp_pdf_.sec, stamp_pdf_.nsec, stamp_pdf_.toSec(), stamp_pose_arrived_.sec, stamp_pose_arrived_.nsec, stamp_pose_arrived_.toSec());
        } else {
            desyncedMessage("pdf");
        }
    }
    
    void poseBestCb(geometry_msgs::PoseStampedConstPtr msg) {
        ROS_DEBUG("got pose_best");
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
        ROS_DEBUG("got map_divided");
        if (state==INFERENCE) {
            stamp_map_divided_ = msg->header.stamp;
            stats_.t_div += stamp_map_divided_-stamp_pose_best_;
            //ROS_INFO("*****>>>> added to t_drive=%f, (%d , %d)=%f - (%d , %d)=%f", (stamp_map_divided_-stamp_pose_best_).toSec(), stamp_map_divided_.sec, stamp_map_divided_.nsec, stamp_map_divided_.toSec(), stamp_pose_best_.sec, stamp_pose_best_.nsec, stamp_pose_best_.toSec());
            stats_.t_drive += stamp_map_divided_-stamp_pose_best_;
        } else {
            desyncedMessage("map_divided");
        }
    }

    void onPoiReached() {
        state = WAIT4POI;

        ROS_DEBUG("Real path to POI: ");
        printPath(waypoints_);
        double dl = calculatePathLength(waypoints_);
        ROS_DEBUG("Real path length to POI: %f", dl);
        stats_.l_real += dl;
        stats_.waypts += waypoints_.size()-1;
        ROS_DEBUG("-------------------Added number of waypts: %d", (int)(waypoints_.size()-1));
        
        waypoints_[0] = waypoints_.back();
        waypoints_.resize(1);
        stats_.t_nav += stamp_pose_arrived_-stamp_pose_intended_;
        stats_.t_drive += stamp_pose_arrived_-stamp_pose_inferred_;
        //ROS_INFO(">>>>>> (%d , %d)=%f - (%d , %d)=%f", stamp_pose_arrived_.sec, stamp_pose_arrived_.nsec, stamp_pose_arrived_.toSec(), stamp_pose_inferred_.sec, stamp_pose_inferred_.nsec, stamp_pose_inferred_.toSec());
        stamp_cmd_detected_=ros::Time(0);
        ROS_DEBUG("=====arrived to POI ====");
    }
    
    void poseArrivedCb(geometry_msgs::PoseStampedConstPtr msg) {
        ROS_DEBUG("got pose_arrived");
        if (state==INFERENCE || state==DRIVING) {
            stamp_pose_arrived_ = msg->header.stamp;
            Point wp;
            updateVertex(msg->pose, wp.x, wp.y);
            waypoints_.push_back(wp);
            if (state==DRIVING) {
                onPoiReached();
            } else {
                stats_.t_drinf += stamp_pose_arrived_-stamp_map_divided_;
                //ROS_INFO("--->>>> added to t_dvive=%f from  (%d , %d)=%f - (%d , %d)=%f", (double)(stamp_pose_arrived_-stamp_map_divided_).toSec(), stamp_pose_arrived_.sec, stamp_pose_arrived_.nsec, stamp_pose_arrived_.toSec(), stamp_map_divided_.sec, stamp_map_divided_.nsec, stamp_map_divided_.toSec());
                ROS_DEBUG("== arrived to waypoint ==");
            }
        } else {
            desyncedMessage("pose_arrived");
        }
    }
    
    void cmdIntendedCb(CommandConstPtr msg) {
        ROS_DEBUG("got cmd_intended");
        if (state==INFERENCE) {
            cmd_intended_ = msg->cmd;
            stats_.dcs_total++;
            ROS_DEBUG("+++++++++ stats_.dcs_total=%d", stats_.dcs_total);
        } else {
            desyncedMessage("cmd_intended");
        }
    }

    void cmdDetectedCb(CommandConstPtr msg) {
        ROS_DEBUG("got cmd_detected");
        if (state==INFERENCE) {
            stamp_cmd_detected_ = msg->header.stamp;
            stats_.t_inf += stamp_cmd_detected_-stamp_pose_arrived_;
            //ROS_INFO(">>>>>> added to t_inf=%f : (%d , %d)=%f - (%d , %d)=%f", (stamp_cmd_detected_-stamp_pose_arrived_).toSec(), stamp_cmd_detected_.sec, stamp_cmd_detected_.nsec, stamp_cmd_detected_.toSec(), stamp_pose_arrived_.sec, stamp_pose_arrived_.nsec, stamp_pose_arrived_.toSec());
            if (msg->cmd != cmd_intended_)
                stats_.dcs_wrong++;
        } else {
            desyncedMessage("cmd_detected");
        }
    }
    
    void poseInferredCb(geometry_msgs::PoseStampedConstPtr msg) {
        ROS_DEBUG("= got pose_inferred =");
        if (state==INFERENCE) {
            state = DRIVING;
            stamp_pose_inferred_ = msg->header.stamp;
            Point wp;
            updateVertex(msg->pose, wp.x, wp.y);
            if (wp.x!=pois_.back().x || wp.y!=pois_.back().y ) {//'2': [[188, 296], [125, 137], [84, 196]],
                ROS_ERROR("poi=(%d,%d), pt=(%d,%d)", pois_.back().x, pois_.back().y, wp.x, wp.y);
                stats_.poi_wrong++;
            }
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
