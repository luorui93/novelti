/*
*/

#include <iostream>
#include <fstream>
#include <string>

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
    
    void paramsCb(std_msgs::String& msg) { 
        
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
    
    void mapCb(IntMap& msg) { 
        if (state==WAIT4POI) {
            state = INFERENCE;
            resolution_ = msg.info.resolution;
            if (!first_run_)
                writeTableRow();
            first_run_ = false;
        }
    }
    
    void poseCurrentCb(geometry_msgs::PoseStamped& msg) { 
        if (!init_pose_defined_) {
            init_pose_defined_ = true;
            int x,y;
            updateVertex(msg.pose, x, y);
            pois_.push_back(Point(x,y));
            waypoints_.push_back(Point(x,y));
        }
    }
    
    void poseIntendedCb(geometry_msgs::PoseStamped& msg) { 
        stamp_cmd_intended_ = msg.header.stamp;
    }
    
    void pdfCb(FloatMap& msg) {
        if (state==INFERENCE) {
            stamp_pdf_ = msg.header.stamp;
            record_.dur_calc_pdf += stamp_pdf_-stamp_cmd_intended_;
        } else {
            desyncedMessage("pdf");
        }
    }
    
    void poseBestCb(geometry_msgs::PoseStamped& msg) { 
        if (state==INFERENCE) {
            stamp_pose_best_ = msg.header.stamp;
            record_.dur_calc_best_pose += stamp_pose_best_-stamp_pdf_;
            //Point wp;
            //updateVertex(msg.pose, wp.x, wp.y);
        } else {
            desyncedMessage("pose_best");
        }
    }
    
    void mapDividedCb(IntMap& msg) { 
        if (state==INFERENCE) {
            stamp_map_divided_ = msg.header.stamp;
            record_.dur_calc_map_div += stamp_map_divided_-stamp_pose_best_;
            record_.dur_driving += stamp_map_divided_-stamp_pose_best_;
        } else {
            desyncedMessage("map_divided");
        }
    }

    void poseArrivedCb(geometry_msgs::PoseStamped& msg) { 
        if (state==INFERENCE || state==DRIVING) {
            stamp_pose_arrived_ = msg.header.stamp;
            Point wp;
            updateVertex(msg.pose, wp.x, wp.y);
            waypoints_.push_back(wp);
            if (state==DRIVING) {
                state = WAIT4POI;
                record_.length_real += calculateLengthToPOI();
                waypoints_.resize(0);
                waypoints_.push_back(wp);
                record_.dur_driving += stamp_pose_arrived_-stamp_pose_inferred_;
            } else {
                record_.dur_drinference += stamp_pose_arrived_-stamp_map_divided_;
            }
        } else {
            desyncedMessage("pose_arrived");
        }
    }
    
    void cmdIntendedCb(Command& msg) { 
        if (state==INFERENCE) {
            cmd_intended_ = msg.cmd;
            record_.n_decisions++;
        } else {
            desyncedMessage("cmd_intended");
        }
    }
    
    void cmdDetectedCb(Command& msg) { 
        if (state==INFERENCE) {
            if (msg.cmd != cmd_intended_)
                record_.n_misdetected_decisions++;
        } else {
            desyncedMessage("cmd_detected");
        }
    }
    
    void poseInferredCb(geometry_msgs::PoseStamped& msg) { 
        if (state==INFERENCE) {
            state = DRIVING;
            stamp_pose_inferred_ = msg.header.stamp;
            Point wp;
            updateVertex(msg.pose, wp.x, wp.y);
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
