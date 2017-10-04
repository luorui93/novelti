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

#include <novelti/StartExperiment.h>
#include <novelti/FloatMap.h>
#include <novelti/IntMap.h>
#include <novelti/Command.h>
#include <novelti/common.cpp>

#include "table_records.cpp"

using namespace cwave;
using namespace std;

namespace novelti {

class MsgProcessor {
public:
    enum State { WAIT4POI, INFERENCE, DRIVING };
    State state;
    
    ostream& sout;
    
    bool init_pose_defined_;
    bool first_run_;
    CourseParams prms_;
    CourseStats  stats_;
    CourseStats2 stats2_;
    
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
        ROS_DEBUG("state: ->WAIT4POI");
        prms_.run = ros::Time(0);
        prms_.tryidx = 0;
        pois_.push_back(Point());
        waypoints_.push_back(Point());
        first_run_ = true;
        resetStats();
        writeTableHeader();
    }

    void readMap(IntMapConstPtr msg) { 
        prms_.nverts = 0;
        new (&cmap_) CompoundMap(msg->info.width, msg->info.height);
        //ROS_INFO("w=%d, h=%d", msg->info.width, msg->info.height);
        for (int x=0; x<msg->info.width; x++)
            for (int y=0; y<msg->info.height; y++)
                if (msg->data[x + y*msg->info.width]==255) {
                    cmap_.setPixel(x,y, FREED); //free
                    prms_.nverts++;
                }
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
        sout << "  ";
        stats2_.headerOut(sout);
        sout << endl;
    }
    
    void writeTableRow () {
        prms_.tryidx++;
        stats_.l_ideal = calculatePathLength(pois_);
        stats_.t_sep = (stats_.t_inf+stats_.t_drinf).toSec() + (stats_.l_ideal/prms_.vel);
        updateStats2();
        sout << prms_ << "  " <<  stats_ << "  " << stats2_ << endl;
        //ROS_DEBUG("POIs: (%d,%d), (%d,%d), (%d,%d)", pois_[0].x, pois_[0].y, pois_[1].x, pois_[1].y, pois_[2].x, pois_[2].y);
        resetStats();
    }
    
    void resetStats() {
        init_pose_defined_ = false;
        resolution_ = -1.0;
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
    
    void updateStats2() {
        stats2_.over_len  = stats_.l_real/stats_.l_ideal;
        stats2_.over_time = stats_.t_nav.toSec()/(stats_.l_ideal/prms_.vel);
        stats2_.sep2nav   = stats_.t_sep/stats_.t_nav.toSec();
        stats2_.det_rate  = 100.0 * (stats_.dcs_total-stats_.dcs_wrong) / stats_.dcs_total;
        stats2_.per_calc  = 100.0 * (stats_.t_pdf.toSec() + stats_.t_pos.toSec()) / stats_.t_nav.toSec();
        stats2_.per_drive = 100.0 * stats_.t_drive.toSec()/stats_.t_nav.toSec();
        stats2_.per_drinf = 100.0 * stats_.t_drinf.toSec()/stats_.t_nav.toSec();
        stats2_.per_infer = 100.0 * stats_.t_inf.toSec()/stats_.t_nav.toSec();
    }
    
    void desyncedMessage(const char* msg) {
        cerr << "Message from /" <<msg << " received when it shouldn't have been received\n";
        exit(1);
    }
    
    void paramCb(std_msgs::StringConstPtr msg) {
        YAML::Node prms = YAML::Load(msg->data.c_str());
        prms_.commit  = prms["node_param_publisher"]["commit"].as<string>().substr(0,7); 
        prms_.comp_id = prms["node_param_publisher"]["comp_id"].as<string>().substr(0,6); 
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
            if (!first_run_) {
                writeTableRow();
            }
            resolution_ = msg->info.resolution;
            ROS_DEBUG("resolution = %f", resolution_);
            first_run_ = false;
        }
    }
    
    void poseCurrentCb(geometry_msgs::PoseStampedConstPtr msg) {
        //ROS_DEBUG("got pose_current: (%f,%f)", msg->pose.position.x, msg->pose.position.y);
        if (!init_pose_defined_ && resolution_>0.0) {
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
            ROS_DEBUG("state: WAIT4POI->INFERENCE");
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
        ROS_DEBUG("state: ->WAIT4POI");

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
            //ROS_DEBUG("+++++++++ stats_.dcs_total=%d", stats_.dcs_total);
        } else {
            desyncedMessage("cmd_intended");
        }
    }

    void cmdDetectedCb(CommandConstPtr msg) {
        ROS_DEBUG("got cmd_detected");
        if (state==INFERENCE) {
            stamp_cmd_detected_ = msg->header.stamp;
            if (stamp_pose_arrived_!= ros::Time(0)) {
                /* Theoretically, stamp_pose_arrived_ should never be 0, because due to
                 * best_pose_finder's safety_coef<1.0 /pose_arrived should always come before /cmd_detected
                 * However sometimes (not very often) in bag files that's not true, for some reason.
                 * This can be addressed later, for now, just don't increase t_inf 
                 * if pose_arrived has not yet arrived (stamp_pose_arrived==0) */
                stats_.t_inf += stamp_cmd_detected_-stamp_pose_arrived_;
                ROS_DEBUG("added to t_inf=%f : (%d , %d)=%f - (%d , %d)=%f", (stamp_cmd_detected_-stamp_pose_arrived_).toSec(), stamp_cmd_detected_.sec, stamp_cmd_detected_.nsec, stamp_cmd_detected_.toSec(), stamp_pose_arrived_.sec, stamp_pose_arrived_.nsec, stamp_pose_arrived_.toSec());
            }
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
            ROS_DEBUG("state: INFERENCE->DRIVING");
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
