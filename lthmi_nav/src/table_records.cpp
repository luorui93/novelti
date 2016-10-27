#include <iostream>
#include <fstream>
#include <string>

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
    long        nverts; //static doc doc_nvts     = "Number of reachable vertices on the map";

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
            "%22s  %5s  %3s  %22s  %10s  "
            "%16s  %5s  %6s  %9s"
            "%12s  %6s  %7s  %7s  "
            "%5s  %5s  %8s  "
            "%16s  %5s  "
            "%12s  %10s  "
            "%3s  %10s  %6s"
        ) %
        "run" % "tries" % "try" % "start" % "commit" % 
        "map" % "path"  % "resol" % "nverts" %
        "mx" % "period" % "vel" % "trobot" % 
        "phigh" % "plow" % "peps" % 
        "pos" % "ksafe" % 
        "div" % "popt" % 
        "bag" % "rviz" % "delay";
    }
};

ostream& operator<<(ostream& out, const CourseParams& r) {
    char s = CourseParams::s;
    return out << boost::format(
            // run   trs  try  strt  cmmt  map   pth  resol  mx   perd   vel    trobo  phigh  plow   eps    pos   ksafe  div   popt  bag  rviz  delay;
            "%22f  %5d  %3d  %22f  %10s  "
            "%16s  %5d  %6.3f  %9ld"
            "%12s  %6.4f  %7.4f  %7.5f  "
            "%5.3f  %5.3f  %8.2e  "
            "%16s  %5.3f  "
            "%12s  %10s  "
            "%3d  %10s  %6.4f"
        ) %
        r.run % r.tries % r.tryidx % r.start % r.commit % 
        r.map  % r.path % r.resol % r.nverts %
        r.mx % r.period % r.vel % r.trobot % 
        r.phigh % r.plow % r.peps % 
        r.pos % r.ksafe % 
        r.div % r.popt % 
        r.bag % r.rviz % r.delay;
}




class CourseStats : public Record {
public:
    double   l_ideal;   //"Length of the shortest path connecting all POIs in the order they are given"
    double   l_real;    //"Length of the actual path made by the robot to visit all POIs"
    
    int      dcs_total; //"Total number of decisions the user had to make while proceeding thorugh this course"
    int      dcs_wrong; //"Number of decisions that have been incorrectly detected"
    int      poi_total; //"Total number of POIs visited in this course"
    int      poi_wrong; //"Number of POIs that were inferred incorrectly"
    int      waypts;    //"Total number of waypoints (includes POIs as well): waypts = dcs_total+poi_total (extra waypoints is added on every POI)"
    
    duration t_inf;     //"The accumulated amount of time spent purely on inference"
    duration t_drive;   //"The accumulated amount of time spent purely on driving"
    duration t_drinf;   //"The accumulated amount of time spent on simulteneous driving and inference"
    duration t_pdf;     //"The accumulated amount of time spent calculating PDFs"
    duration t_pos;     //"The accumulated amount of time spent on search for bet pose"
    duration t_div;     //"The accumulated amount of time spent on map division"
    duration t_nav;     //"Total navigation time: t_nav = t_pdf + t_pos + t_drive + t_drinf + t_inf (t_nav = arrived_t(DRIVING)-pose_intended_t)"
    
    void headerOut(ostream& out) {
        out << boost::format(
                "%12s  %12s  "
                "%9s  %9s  %9s  %9s  %6d  "
                "%18s  %18s  %18s  %18s  %18s  %18s  %18s"
            ) %
            "l_ideal" % "l_real" % 
            "dcs_total" % "dcs_wrong" % "poi_total" % "poi_wrong" % "waypts" % 
            "t_nav" % "t_inf" % "t_drive" % "t_drinf" % "t_pdf" % "t_pos" % "t_div";
    }
};

ostream& operator<<(ostream& out, const CourseStats& v) {
    return out << boost::format(
          // l_idea  l_real  d_t  d_w  p_t  p_w  wpts t_inf   t_driv  t_drnf  t_pdf   t_pos   t_div
            "%12.3f  %12.3f  "
            "%9d  %9d  %9d  %9d  %6d  "
            "%18.6f  %18.6f  %18.6f  %18.6f  %18.6f  %18.6f  %18.6f"
        ) %
        v.l_ideal % v.l_real % 
        v.dcs_total % v.dcs_wrong % v.poi_total  % v.poi_wrong % v.waypts % 
        v.t_nav % v.t_inf % v.t_drive % v.t_drinf % v.t_pdf % v.t_pos % v.t_div;
}



class CourseStats2 : public Record {
public:
    double over_len;  //l_real/l_ideal
    double over_time; //t_nav/t_ideal = t_nav/(l_ideal/vel);
    double det_rate; // 100% * dcs_correct/dcs_total = (dcs_total-dcs_wrong)/dcs_total

    double per_calc;  // 100% * (t_pdf + t_pos)/t_nav
    double per_drive; // 100% * t_drive/t_nav
    double per_drinf; // 100% * t_drinf/t_nav
    double per_infer; // 100% * t_inf/t_nav
    
    void headerOut(ostream& out) {
        out << boost::format(
                "%9s  %9s  %9s  "
                "%9s  %9s  %9s  %9s"
            ) %
            "over_len" % "over_time" % "det_rate" % 
            "per_calc" % "per_drive" % "per_drinf" % "per_infer";
    }
};
    

ostream& operator<<(ostream& out, const CourseStats2& v) {
    return out << boost::format(
            "%9.3f  %9.3f  %9.3f  "
            "%9.3f  %9.3f  %9.3f  %9.3f"
        ) %
        v.over_len % v.over_time % v.det_rate % 
        v.per_calc % v.per_drive  % v.per_drinf % v.per_infer;
}

} //namespace lthmi_nav
