#include <novelti/position_control.h>
#include <limits>
#include <tf/transform_broadcaster.h>
#include <cmath>

#include "best_pose_finder_opt.cpp"
#include "map_divider_tile.cpp"
#include "map_divider_cwave.cpp"
#include "map_divider_vchess.cpp"
#include <novelti/best_pose_finder_quasi_opt.h>

using namespace ros::this_node; //for getName
using namespace novelti;


PositionControl::PositionControl(ros::NodeHandle& node) :
    node_(node)
{
    node_.param<float>("inf/interest_area_coef", interest_area_thresh_, -1.0);
    node_.param<float>("inf/eps", eps_,          std::numeric_limits<float>::epsilon());
    node_.param<bool> ("inf/reset_pdf_on_new",   reset_pdf_on_new_, true);
    
    std::vector<double> view_sizes_param;
    if (!node_.hasParam("inf/view_sizes")) {
        ROS_ERROR("Parameter view_sizes not configured");
    }
    node_.getParam("inf/view_sizes", view_sizes_param);
    //ROS_INFO("%s: ---------------------------------------------------------------------- view_sizes.size()==%d", getName().c_str(), (int)view_sizes_.size());
    if (view_sizes_param.size()==0) {
        view_sizes_ = {256};
    } else {
        for (int k=0; k<view_sizes_param.size(); k++)
            view_sizes_.push_back((int)(view_sizes_param[k]));
    }

    std::vector<double> smooth_rads_param;
    node_.getParam("inf/smooth_rads", smooth_rads_param);
        if (smooth_rads_param.size() != view_sizes_.size() && smooth_rads_param.size() != 0) {
            ROS_ERROR("ERROR: arrays view_sizes and smooth_rads must have the same length unless smooth_rads is empty");
            throw ros::Exception("Parameter error, see message above");
        } else {
            for (int k=0; k<smooth_rads_param.size(); k++)
                smooth_rads_.push_back(smooth_rads_param[k]);
        }
    
    readPOIsParam();
    position_inferred_.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0); //orientation will be ignored
    best_position_finder_ = newBestPositionFinder();
    map_divider_ = newMapDivider();
}

PositionControl::~PositionControl() {
    delete best_position_finder_;
    delete map_divider_;
}


void PositionControl::start(StartExperiment::Request& req) {
    pdf_ = FloatMap();
    pdf_.header.frame_id = "/map";
    pdf_.info.width = req.map.info.width+1;
    pdf_.info.height = req.map.info.height+1;
    pdf_.info.resolution = req.map.info.resolution;
    pdf_.info.origin.position.x = -0.5*req.map.info.resolution;
    pdf_.info.origin.position.y = -0.5*req.map.info.resolution;
    pdf_.data = std::vector<float>(pdf_.info.width*pdf_.info.height, PDF_UNREACHABLE);
    
    //determine reachable vertices
    int k;
    long int total_vx = 0;
    for (int x=0; x<req.map.info.width-1; x++) {
        for (int y=0; y<req.map.info.height-1; y++) {
            k = x + y*req.map.info.width;
            if  (req.map.data[k]==0 || 
                 req.map.data[k+1]==0 || 
                 req.map.data[k+req.map.info.width]==0 || 
                 req.map.data[k+1+req.map.info.width]==0
            ) {
                pdf_.data[x+1 + (y+1)*pdf_.info.width] = 1.0;
                total_vx++;
            }
        }
    }
    uniform_prob_       = 1.0/total_vx;
    pub_pdf_            = node_.advertise<FloatMap>("/pdf", 1, true); //not latched
    pub_position_inferred_   = node_.advertise<geometry_msgs::PoseStamped>("/position_inferred", 1, true); //latched to make sure topic_tools relay can receive the message.
    best_position_finder_->startExp(req);
    map_divider_ ->startExp(req);
}


void PositionControl::stop() {
    //sub_map_div_.shutdown();
    pub_pdf_.shutdown();
    pub_position_inferred_.shutdown();
    best_position_finder_->stopExp();
    map_divider_ ->stopExp();    
}


void PositionControl::initPriors(std::vector<double>& new_priors) { 
    if (reset_pdf_on_new_) {
        new_pdf_ = true;
        if (pois_.size()==0)
            PdfUtils::setUniform(pdf_.data, uniform_prob_);
        else
            setPdfFromPOIs();
    }
    best_position_finder_->calculate(pdf_);
    map_divider_->divide(pdf_, best_position_finder_->pose_best);
    stats_ = PdfUtils::accumulate<float>(pdf_.data, getIndexMap(), new_priors);
}


void PositionControl::update(const std::vector<double>& coefs, const int cmd, std::vector<double>& new_priors) {
    map_divider_->highlightSelection(cmd);
    new_pdf_ = false;
    PdfUtils::update<float>(pdf_.data, getIndexMap(), coefs);
    if (needsSmoothening())
        smoothen();
    PdfUtils::denullifyNormalize<float>(pdf_.data, eps_);
    
    best_position_finder_->calculate(pdf_);
    map_divider_->divide(pdf_, best_position_finder_->pose_best);
    stats_ = PdfUtils::accumulate<float>(pdf_.data, getIndexMap(), new_priors);
}


void PositionControl::onInferred(int inferredCmd) {
    int y = stats_.max_k / pdf_.info.width;
    int x = stats_.max_k % pdf_.info.width;
    position_inferred_.header.stamp = ros::Time::now();
    position_inferred_.pose.position.x = x*pdf_.info.resolution;
    position_inferred_.pose.position.y = y*pdf_.info.resolution;
    pub_position_inferred_.publish(position_inferred_);
    ROS_INFO("%s: published /pose_inferred, vertex=(%d,%d), pose=(%f,%f)", 
             getName().c_str(), x, y, position_inferred_.pose.position.x, position_inferred_.pose.position.y);    
}

void PositionControl::act() {
    //if (visual)
        publishPdf();
    best_position_finder_->publish();
    map_divider_->publish();
}

const geometry_msgs::PoseStamped& PositionControl::getPositionInferred() const {
    return position_inferred_;
}


const std::vector<int>& PositionControl::getIndexMap() {
    return map_divider_->map_divided.data;
}


void PositionControl::readPOIsParam() {
    XmlRpc::XmlRpcValue pois_param;
    node_.getParam("inf/pois", pois_param);
    if (pois_param.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
        //ROS_ASSERT(pois_param.getType() == XmlRpc::XmlRpcValue::TypeStruct);
        for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = pois_param.begin(); it != pois_param.end(); ++it) {
            auto poi = it->second;
            ROS_ASSERT(poi.getType() == XmlRpc::XmlRpcValue::TypeStruct);
            //ROS_ERROR("POI added");
            pois_.push_back({poi["x"],poi["y"],poi["sigma"],poi["k"]});
        }
    }
}

void PositionControl::setPdfFromPOIs() {
    int k;
    double total = 0.0;
    float p, d, resol=pdf_.info.resolution;
    for (int x=0; x<pdf_.info.width-2; x++) {
        for (int y=0; y<pdf_.info.height-2; y++) {
            k = x + y*pdf_.info.width;
            //k = x + y*(pdf_.info.width-1);
            if (pdf_.data[k]>=0) {
                p = 0.0;
                for (auto& poi: pois_) {
                    d = (resol*x-poi.x)*(resol*x-poi.x) + (resol*y-poi.y)*(resol*y-poi.y);
                    p += poi.k * exp(-d/(2*poi.sigma*poi.sigma));
                }
                pdf_.data[k] = p;
                total += p;
            }
        }
    }
    PdfUtils::normalize<float>(pdf_.data, total);
}



void PositionControl::publishPdf() {
    if (interest_area_thresh_ > 0.0)
        publishViewTf();
    pdf_.header.stamp = ros::Time::now();
    pub_pdf_.publish(pdf_);
    ros::spinOnce();
    ROS_INFO("%s: =========== published pdf, max_prob=%f", getName().c_str(), stats_.max);
}


bool PositionControl::needsSmoothening() {
    return smooth_rads_.size()!=0 && smooth_rads_[view_size_id_]!=0;
}


bool PositionControl::isSmoothableVertex(int cx, int cy, int smooth_rad) {
    /* vertex needs smoothing if it's located close (compared to smooth_rad)
     *      to the boundary between regions */
    int k = cx + cy*pdf_.info.width;
    int cur_region = pdf_.data[k];
    for (int x=std::max(0,cx-smooth_rad); x<=std::min((int)(pdf_.info.width-1), cx+smooth_rad); x++) {
        for (int y=std::max(0,cy-smooth_rad); y<=std::min((int)(pdf_.info.height-1), cy+smooth_rad); y++) {
            k = x + y*pdf_.info.width;
            if (pdf_.data[k] != 255 && pdf_.data[k] != cur_region)
                return true;
        }
    }
    return false;
}

void PositionControl::smoothen() {
    if (smooth_rads_.size()==0 || smooth_rads_[view_size_id_]==0)
        return;
    int smooth_rad = smooth_rads_[view_size_id_];
    ROS_WARN("Smoothening pdf, view_size_id=%d, view_size=%d, smooth_rad = %d", view_size_id_, view_sizes_[view_size_id_], smooth_rad);    
    FloatMap pdf_copy = pdf_;
    int i,n;
    float p, p_avg;
    double total = 0.0;
    for (int cy=0, k=0; cy<pdf_copy.info.height-1; cy++) {
        for (int cx=0; cx<pdf_copy.info.width-1; cx++, k++) {
            if (pdf_copy.data[k] >= 0) {
                if (isSmoothableVertex(cx,cy,smooth_rad)) {
                    //calculate smoothed valued
                    n = 0;
                    p_avg = 0.0;
                    for (int x=std::max(0,cx-smooth_rad); x<=std::min((int)(pdf_copy.info.width-1), cx+smooth_rad); x++) {
                        for (int y=std::max(0,cy-smooth_rad); y<=std::min((int)(pdf_copy.info.height-1), cy+smooth_rad); y++) {
                            p = pdf_copy.data[x + y*pdf_.info.width];
                            if (p>=0) {
                                n++;
                                p_avg += p;
                            }
                        }
                    }
                    pdf_.data[k] = p_avg/n;
                }
                total += pdf_.data[k];
            }
        }
        k++;
    }
    PdfUtils::normalize<float>(pdf_.data, total);
    ROS_WARN("%s: total probability of the PDF after smoothening = %f", 
             getName().c_str(), total);
}


void PositionControl::publishViewTf() {
    float thresh = interest_area_thresh_*(stats_.max-stats_.min)+stats_.min;
    int xmin=std::numeric_limits<int>::max(), xmax=std::numeric_limits<int>::min();
    int ymin=std::numeric_limits<int>::max(), ymax=std::numeric_limits<int>::min();
    for (int y=0, k=0; y<pdf_.info.height-1; y++) {
        for (int x=0; x<pdf_.info.width-1; x++, k++) {
            if (pdf_.data[k] >= thresh) {
                if (x<xmin) 
                    xmin = x;
                if (y<ymin)
                    ymin = y;
                if (x>xmax)
                    xmax = x;
                if (y>ymax)
                    ymax = y;
            }
        }
        k++;
    }
    if (new_pdf_) {
        xmin = 0; 
        ymin = 0;
        xmax = pdf_.info.width-1;
        ymax = pdf_.info.height-1;
    }
        
    //calculate TF from where the interest area will be completely visible
    int view_size;
    int d = std::max({xmax-xmin, ymax-ymin});
    int x=(xmin+xmax)/2;
    int y=(ymin+ymax)/2;
    //std::vector<int> view_sizes_ = {16, 32, 64, 128, 256};
    for (view_size_id_=0; view_size_id_<view_sizes_.size(); view_size_id_++)
        if (view_sizes_[view_size_id_]>=d)
            break;
    if (view_size_id_>=view_sizes_.size())
        view_size_id_--;
    int cr = view_sizes_[view_size_id_]/2;
    
    int view_x = (int)(round(x/cr)*cr);
    int view_y = (int)(round(y/cr)*cr);
    if (view_x==0) 
        view_x=cr;
    if (view_y==0) 
        view_y=cr;

    
    if ((xmin < view_x-cr || xmax> view_x+cr || ymin <view_y-cr || ymax>view_y+cr) && view_size_id_ < view_sizes_.size()-1) {
        view_size_id_++;
        cr = view_sizes_[view_size_id_]/2;
    }
    view_x = (int)(round(x/cr)*cr);
    view_y = (int)(round(y/cr)*cr);
    if (view_x==0) 
        view_x=cr;
    if (view_y==0) 
        view_y=cr;

    double cam_distance = (1.5  -  0.35*(view_sizes_[view_size_id_]-20)/250.0)*view_sizes_[view_size_id_]*pdf_.info.resolution;//2+2*(int(max_area_size*0.2)/2);
    double cam_x = pdf_.info.resolution*view_x;
    double cam_y = pdf_.info.resolution*view_y;
    /*if (true || cam_distance==8) {
        cam_x = w/2;
        cam_y = h/2;
    }*/
    static tf::TransformBroadcaster view_tf;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(cam_x,cam_y, cam_distance) );
    tf::Quaternion q;
    q.setRPY(0.0, M_PI/2, M_PI/2);
    transform.setRotation(q);
    view_tf.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/autocam"));
}


BestPoseFinder* PositionControl::newBestPositionFinder() {
    std::string method;   
    node_.getParam("pos/method", method);
    if      (method =="no_move")
        return new BestPoseFinder("pos");
    else if (method =="ra_maxprob")   
        return new QuasiOptPoseFinder(QuasiOptPoseFinder::RA_MAXPROB,"pos");
    else if (method =="maxprob_euc")  //doesn't guarantee convergance
        return new QuasiOptPoseFinder(QuasiOptPoseFinder::MAXPROB_EUC,"pos");
    else if (method =="maxprob_obst") 
        return new QuasiOptPoseFinder(QuasiOptPoseFinder::MAXPROB_OBST,"pos");
    else if (method =="cog_euq")      //doesn't guarantee convergance
        return new QuasiOptPoseFinder(QuasiOptPoseFinder::COG_EUC,"pos");
    else if (method =="nearcog_euc")  //doesn't guarantee convergance
        return new QuasiOptPoseFinder(QuasiOptPoseFinder::NEARCOG_EUC,"pos");
    else if (method =="nearcog_obst") 
        return new QuasiOptPoseFinder(QuasiOptPoseFinder::NEARCOG_OBST,"pos");
    else if (method =="cog2lopt")
        return new OptPoseFinder(OptPoseFinder::COG2LOPT,"pos");
    else if (method =="ramaxprob2lopt")
        return new OptPoseFinder(OptPoseFinder::RAMAXPROB2LOPT,"pos");
    else if (method =="maxprob2lopt")     
        return new OptPoseFinder(OptPoseFinder::MAXPROB2LOPT,"pos");
    else if (method =="gopt")
        return new OptPoseFinder(OptPoseFinder::GOPT,"pos");
    else {
         ROS_ERROR("%s: wrong value for 'pos/method ' parameter ('%s'), will die now", getName().c_str(), method .c_str());
    }
}



MapDivider* PositionControl::newMapDivider() {
    std::string method;   
    node_.getParam("div/method", method);
    //map_divider initialization
    if      (method=="vtile")        
        return new TileMapDivider(TileMapDivider::VERT,"div");
    else if (method=="htile")        
        return new TileMapDivider(TileMapDivider::HORIZ,"div");
    else if (method=="altertile")    
        return new TileMapDivider(TileMapDivider::ALTER,"div");
    else if (method=="equidist")     
        return new CWaveMapDivider(CWaveMapDivider::EQUIDIST,"div");
    else if (method=="extremal")     
        return new CWaveMapDivider(CWaveMapDivider::EXTREMAL,"div");
    else if (method=="extredist")    
        return new CWaveMapDivider(CWaveMapDivider::EXTREDIST,"div");
    else if (method=="vchess")       
        return new VertChessMapDivider("div");
    else if (method=="nearcog_extremal") 
        return new CWaveMapDivider(CWaveMapDivider::NEARCOG_EXTREMAL,"div");
     else { 
         ROS_ERROR("%s: wrong value for 'div/method' parameter ('%s'), will die now", getName().c_str(), method.c_str());
     }
}