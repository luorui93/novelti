/*
       subs                                            pubs
                   +----------------------------+
                   |                            |
/cmd_detected ---> |                            | ---> /pdf
 /map_divided ---> |      inference_unit        |
                   |                            |
                   +----------------------------+
                         ^                ^
                         |                |
                     srv: start         srv: new_goal
                         req:  scene        req:  -
                         resp: -            resp: -

----------------------------------------------------------------------------------------

                                   +-----------+
           max_prob <= thresh_low  |           | max_prob >= thresh_high
         +------------------------>| INFERRING +--------------------------+
         |                         |           |                          |
         |                         +-----------+                          |
         |                                                                v
 +----------------+                                               +----------------+
 |                |                      new_goal service req     |                |
 | INFERRING_NEW  |<----------------------------------------------+    INFERRED    |
 |                |                                               |                |
 +----------------+                                               +----------------+
*/

#include <novelti/inference_unit.h>
#include <limits>
#include <tf/transform_broadcaster.h>

using namespace novelti;


InferenceUnit::InferenceUnit(const std::string paramPrefix) :
    isNode(false),
    node("~")
{
    node.param<float>("inf/interest_area_coef", interest_area_thresh_, -1.0);
    node.param<float>("inf/thresh_high", thresh_high, 0.98);
    node.param<float>("inf/thresh_low", thresh_low, 0.5);
    node.param<float>("inf/eps", eps, std::numeric_limits<float>::epsilon());
    node.param<bool>("inf/reset_pdf_on_new", reset_pdf_on_new_, false);
    node.param<bool>("inf/check_sync", check_sync_, true);
    node.param<float>("ori/orientation_resolution", orientation_resol, 5.0);

    //ROS_INFO("%s: ---------------------------------------------------------------------- smoothen=%d", getName().c_str(), smoothen_ ? 1 :0);
    
    std::vector<double> view_sizes;
    if (!node.hasParam("inf/view_sizes")) {
        ROS_ERROR("Parameter view_sizes not configured");
    }
    node.getParam("inf/view_sizes", view_sizes);
    ROS_INFO("%s: ---------------------------------------------------------------------- view_sizes.size()==%d", getName().c_str(), (int)view_sizes.size());
    if (view_sizes.size()==0) {
        view_sizes_ = {256};
    } else {
        for (int k=0; k<view_sizes.size(); k++)
            view_sizes_.push_back((int)(view_sizes[k]));
    }
        
    std::vector<double> smooth_rads;
    node.getParam("inf/smooth_rads", smooth_rads);
        if (smooth_rads.size() != view_sizes_.size() && smooth_rads.size() != 0) {
            ROS_ERROR("ERROR: arrays view_sizes and smooth_rads must have the same length unless smooth_rads is empty");
            throw ros::Exception("Parameter error, see message above");
        } else {
            for (int k=0; k<smooth_rads.size(); k++)
            smooth_rads_.push_back(smooth_rads[k]);
        }
    node.getParam("pois", pois_);
    node.getParam("interface_matrix", interface_matrix);
    n_cmds = (int)floor(sqrt(interface_matrix.size()));
    if (interface_matrix.size()==0 && n_cmds*n_cmds==interface_matrix.size()) {
        ROS_ERROR("ERROR: interface_matrix parameter should be specified, have length>0, and its length has to be a square of an integer number (number of commands)");
        throw ros::Exception("Parameter error, see message above");
    }
//      for (int x=0; x<4; x++)
//          for (int y=0; y<4; y++){
//              double q = interface_matrix[x + n_cmds*y];
//              ROS_INFO("%f", q);}
    priors     = std::vector<double>(n_cmds, 0.0);
    posteriors = std::vector<double>(n_cmds, 0.0);
    coefs      = std::vector<double>(n_cmds, 0.0);
}

InferenceUnit::InferenceUnit():
    InferenceUnit("")
{
    isNode = true;
    ROS_INFO("Inference Unit is running as a node");
}

bool InferenceUnit::srvNewGoal(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp) {
    state = INFERRING; //_NEW;
    // ROS_INFO("%s: new_goal service request received. State INFERRING -> INFERRING_NEW", getName().c_str());
    if (reset_pdf_on_new_) {
        resetPdf();
    } else {
        denullifyPdf(pdf.data);
    }
    setUniformOrientationPdf();
    updateInferenceState();
    //ROS_INFO("%s: new_pdf == %s", getName().c_str(), new_pdf_ ? "True" : "False");
    pubPdf();
    return true;
}

void InferenceUnit::stopExp() {
    sub_map_div.shutdown();
    sub_cmd.shutdown();
    pub_pdf.shutdown();
    pub_position_inf.shutdown();
    pub_pose_inf.shutdown();

}

void InferenceUnit::startExp(novelti::StartExperiment::Request& req) {
    state = INFERRING;
    fast_state = RCVD_NONE;
    pdf = FloatMap();
    pdf.header.frame_id = "/map";
    pdf.info.width = req.map.info.width+1;
    pdf.info.height = req.map.info.height+1;
    pdf.info.resolution = req.map.info.resolution;
    pdf.info.origin.position.x = -0.5*req.map.info.resolution;
    pdf.info.origin.position.y = -0.5*req.map.info.resolution;
    pdf.data = std::vector<float>(pdf.info.width*pdf.info.height, PDF_UNREACHABLE);
    opdf = OrientationPdf();
    opdf.header.frame_id = "/orientation_display";
    opdf.data = std::vector<float>(360 / orientation_resol,0.0);
    norm_pdf = pdf;
    int k;
    long int total_vx = 0;

    //determine reachable vertices
    for (int x=0; x<req.map.info.width-1; x++) {
        for (int y=0; y<req.map.info.height-1; y++) {
            k = x + y*req.map.info.width;
            if (req.map.data[k]==0 || req.map.data[k+1]==0 || req.map.data[k+req.map.info.width]==0 || req.map.data[k+1+req.map.info.width]==0) {
                pdf.data[x+1 + (y+1)*pdf.info.width] = 1.0;
                total_vx++;
            }
        }
    }

    //set uniform pdf over reachable vertices
    uniform_prob_ = 1.0/total_vx;
    // interest_area_thresh_ *= uniform_prob_;
    resetPdf(); //Already reset in start
    setUniformOrientationPdf();
    pub_pdf      = node.advertise<FloatMap>("/pdf", 1, true); //not latched
    pub_opdf     = node.advertise<OrientationPdf>("/opdf", 1, true);
    pub_position_inf = node.advertise<geometry_msgs::PoseStamped>("/position_inferred", 1, false); //not latched
    pub_pose_inf = node.advertise<geometry_msgs::PoseStamped>("/pose_inferred", 1, false);
    if (isNode) {
        sub_map_div  = node.subscribe("/map_divided", 1, &InferenceUnit::mapDivCallback, this);
        sub_cmd      = node.subscribe("/cmd_detected", 1, &InferenceUnit::cmdCallback, this);
    }
    if (interest_area_thresh_ > 0.0)
        publishViewTf();
    if (isNode) {
        srv_new_goal = node.advertiseService("new_goal", &InferenceUnit::srvNewGoal, this);
    }
}

void InferenceUnit::resetPdf() {
    ROS_INFO("%s: resetting pdf", getName().c_str());
    new_pdf_ = true;
    if (pois_.size()==0)
        setUniformPdf();
    else
        setStaticPredictedPdf();
}

void InferenceUnit::setUniformPdf() {
    for (int k=pdf.data.size()-1;k>=0; k--)
        if (pdf.data[k] >=0 ) {
            pdf.data[k] = uniform_prob_;
            norm_pdf.data[k] = pdf.data[k];
            max_prob_k = k;
        }
    max_prob = uniform_prob_;
}

void InferenceUnit::setStaticPredictedPdf() {
    int k;
    double total = 0.0;
    float p, d, poi_x, poi_y, poi_sigma, poi_k;
    for (int x=0; x<pdf.info.width-2; x++) {
        for (int y=0; y<pdf.info.height-2; y++) {
            k = x + y*pdf.info.width;
            //k = x + y*(pdf.info.width-1);
            if (pdf.data[k]>=0) {
                p = 0.0;
                //pois_: x1, y1, sigma1, k1,    x2, y2, sigma2, k2, ...
                for (int i=0; i<pois_.size(); i+=4) {
                    poi_x = pois_[i];
                    poi_y = pois_[i+1];
                    poi_sigma = pois_[i+2];
                    poi_k = pois_[i+3];
                    d = (x-poi_x)*(x-poi_x) + (y-poi_y)*(y-poi_y);
                    p += poi_k * exp(-d/(2*poi_sigma*poi_sigma));
                }
                pdf.data[k] = p;
                norm_pdf.data[k] = pdf.data[k];
                total += p;
            }
        }
    }
    
    
    //normalize
    double total2 = 0.0;
    max_prob = 0;
    for (int k=pdf.data.size()-1; k>=0; k--) { //normalize
        p = pdf.data[k];
        if (p >=0 ) {
            pdf.data[k] = p/total;
            total2 += pdf.data[k];
            if (pdf.data[k]>=max_prob) {
                max_prob = pdf.data[k];
                max_prob_k = k;
            }
        }
    }
    ROS_INFO("%s: Accumulated probability of the whole PDF = %f, total before normalization=%f", getName().c_str(), total2, total);
}

void InferenceUnit::denullifyPdf(std::vector<float>& pdf) { //replaces small probs (<eps) with eps, and normalizes
    double p, sc=0.0, sa=0.0;
    //ROS_INFO("++++++++++++++++ eps=%f", eps);
    for (int k=pdf.size()-1; k>=0; k--) { //replace small probs (<eps) with eps 
        p = pdf[k];
        if (p >=0 && p < eps) {
            sc += eps;
            sa += p;
        }
    }
    double c=(1.0-sc)/(1.0-sa);
    //ROS_INFO("++++++++++++++++ c=%f", c);
    for (int k=pdf.size()-1; k>=0; k--) { //normalize
        p = pdf[k];
        if (p >=0) {
            pdf[k] = p<eps ? eps : c*p;
        }
    }
    
    double min_prob = 1.0;
    for (int k=pdf.size()-1; k>=0; k--) //find min prob
        if (pdf[k]>=0.0 && pdf[k]<min_prob)
            min_prob = pdf[k];
    //ROS_INFO("++++++++++++++++ min_prob=%f", min_prob*1000000);
}

    
void InferenceUnit::calcPriors() {
    std::fill(priors.begin(), priors.end(), 0.0); //priors = 0-vector
    for (int k=0; k<map_divided->data.size(); k++)
        if (pdf.data[k]>=0)
            priors[map_divided->data[k]] += pdf.data[k];
    ROS_INFO("%s: priors from map_divided: [%f, %f, %f, %f]", getName().c_str(), priors[0], priors[1], priors[2], priors[3]);
}

void InferenceUnit::calcUpdCoefs() {
    std::fill(posteriors.begin(), posteriors.end(), 0.0); //posteriors = 0-vector
    double total = 0.0;
    for (int k=0; k<n_cmds; k++) {
        //ROS_INFO(">>>>>>>>>>>>>>> mx=%f, pr=%f", interface_matrix[cmd_detected->cmd + k*n_cmds], priors[k]);
        posteriors[k] = interface_matrix[cmd_detected->cmd + k*n_cmds] * priors[k]; 
        total += posteriors[k];
    }
    //ROS_INFO("%s: calculated posteriors before normalization: [%f, %f, %f, %f]", getName().c_str(), posteriors[0], posteriors[1], posteriors[2], posteriors[3]);
    
    //normalize posteriors 
    for (int k=0; k<n_cmds; k++)
        posteriors[k] = posteriors[k]/total;
    ROS_INFO("%s: calculated posteriors after normalization: [%f, %f, %f, %f]", getName().c_str(), posteriors[0], posteriors[1], posteriors[2], posteriors[3]);
    
    //caluclate update coefficients
    for (int k=0; k<n_cmds; k++) 
         coefs[k] = priors[k] != 0.0 ? posteriors[k]/priors[k] : 0.0;
    //ROS_INFO("%s: coefs: [%f, %f, %f, %f]", getName().c_str(), coefs[0], coefs[1], coefs[2], coefs[3]);
}


void InferenceUnit::mapDivCallback(novelti::IntMapConstPtr msg){
    ROS_INFO("%s: received map_divided (SEQ=%d)", getName().c_str(), msg->header.seq);
    map_divided = msg;
    calcPriors();
    if (fast_state==RCVD_CMD) {
        fast_state = RCVD_NONE;
        updatePdfAndPublish();
    } else {
        fast_state = RCVD_MAPDIV;
        //ROS_INFO("%s: fast_state := RCVD_MAPDIV", getName().c_str());
    }
}
void InferenceUnit::cmdCallback(CommandConstPtr msg){
    ROS_INFO("%s: received cmd_detected = %d (SEQ=%d)", getName().c_str(), msg->cmd, msg->header.seq);
    cmd_detected = msg;

}

void InferenceUnit::noveltiInfCallback(novelti::IntMapConstPtr ptr_map, CommandConstPtr ptr_cmd){
    map_divided = ptr_map;
    cmd_detected = ptr_cmd;
    calcPriors();
    updatePdfAndPublish();
}

void InferenceUnit::updatePdf() {
    calcUpdCoefs();
    max_prob = 0.0;
    min_prob = 1.0;
    double total_prob = 0.0;
    double p;
    for (int k=0; k<pdf.data.size(); k++) {
        p = pdf.data[k];
        if (p>=0) {
            pdf.data[k]=p*coefs[map_divided->data[k]];
            total_prob += pdf.data[k];
            ///ROS_INFO("%s: updating pdf[%d]: %f->%f", getName().c_str(), k, p, pdf.data[k]);
            if (pdf.data[k] >= max_prob) {
                max_prob   = pdf.data[k];
                max_prob_k = k;
            }
            if (pdf.data[k] <= min_prob) {
                min_prob = pdf.data[k];
            }
        }
    }
    smoothenPdf();
    //ROS_INFO("%s: total prob=%f", getName().c_str(), total_prob);
}


bool InferenceUnit::doesNeedSmoothing(int cx, int cy, int smooth_rad) {
    /* vertex needs smoothing if it's located close (compared to smooth_rad)
     *      to the boundary between regions */
    int k = cx + cy*map_divided->info.width;
    int cur_region = map_divided->data[k];
    for (int x=std::max(0,cx-smooth_rad); x<=std::min((int)(map_divided->info.width-1), cx+smooth_rad); x++) {
        for (int y=std::max(0,cy-smooth_rad); y<=std::min((int)(map_divided->info.height-1), cy+smooth_rad); y++) {
            k = x + y*map_divided->info.width;
            if (map_divided->data[k] != 255 && map_divided->data[k] != cur_region)
                return true;
        }
    }
    return false;
}

void InferenceUnit::smoothenPdf() {
    if (smooth_rads_.size()==0)
        return;
    int smooth_rad = smooth_rads_[view_size_id_];
    //int smooth_rad = (int)(round(168894.6 + (-0.7231753 - 168894.6)/(1 + pow((view_size_/426557900.0),0.7082083))));
    //(int)round(0.02*view_size_ + 0.6); // 256->6, 16->1  
    ROS_WARN("Smoothening pdf, view_size_id=%d, view_size=%d, smooth_rad = %d", view_size_id_, view_sizes_[view_size_id_], smooth_rad);    
    if (smooth_rad==0)
        return;
    FloatMap pdf_copy = pdf;
    //int smooth_rad = (int)(round(6.464102 + (-0.4641016 - 6.464102)/(1 + pow((view_size_/64.0),1.899969))));
    //int smooth_rad = (int)(round(220.8386 + (-203.4252 - 220.8386)/(1 + pow((view_size_/79986.57),0.009532408))));
    //int smooth_rad = (int)(round(3.523462 + (-0.1923772 - 3.523462)/(1 + pow((view_size_/102.2199),1.923313))));
    /*int smooth_rad = 0;
    switch (view_size_) {
        case  16: smooth_rad=0; break;
        case  32: smooth_rad=0; break;
        case  64: smooth_rad=1; break;
        case 128: smooth_rad=1; break;
        case 256: smooth_rad=1; break;
    }*/
    
    int i,n;
    float p, p_avg;
    double total = 0.0;
    for (int cy=0, k=0; cy<pdf_copy.info.height-1; cy++) {
        for (int cx=0; cx<pdf_copy.info.width-1; cx++, k++) {
            if (pdf_copy.data[k] >= 0) {
                if (doesNeedSmoothing(cx,cy,smooth_rad)) {
                    //calculate smoothed valued
                    n = 0;
                    p_avg = 0.0;
                    for (int x=std::max(0,cx-smooth_rad); x<=std::min((int)(pdf_copy.info.width-1), cx+smooth_rad); x++) {
                        for (int y=std::max(0,cy-smooth_rad); y<=std::min((int)(pdf_copy.info.height-1), cy+smooth_rad); y++) {
                            p = pdf_copy.data[x + y*pdf.info.width];
                            if (p>=0) {
                                n++;
                                p_avg += p;
                            }
                        }
                    }
                    pdf.data[k] = p_avg/n;
                }
                total += pdf.data[k];
            }
        }
        k++;
    }
    
    //normalize
    double total2 = 0.0;
    for (int k=pdf.data.size()-1; k>=0; k--) { //normalize
        p = pdf.data[k];
        if (p >=0 ) {
            pdf.data[k] = p/total;
            total2 += pdf.data[k];
        }
    }
    
    ROS_WARN("%s: total probability of the PDF after smoothening = %f, and then after normalizarion = %f", getName().c_str(), total, total2);
}

void InferenceUnit::publishViewTf() {
    float thresh = interest_area_thresh_*(max_prob-min_prob)+min_prob;
    int xmin=std::numeric_limits<int>::max(), xmax=std::numeric_limits<int>::min();
    int ymin=std::numeric_limits<int>::max(), ymax=std::numeric_limits<int>::min();
    for (int y=0, k=0; y<pdf.info.height-1; y++) {
        for (int x=0; x<pdf.info.width-1; x++, k++) {
            if (pdf.data[k] >= thresh) {
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
        xmax = pdf.info.width-1;
        ymax = pdf.info.height-1;
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


    /*
     * //[xmin, xmax]=[158,173], [ymin,ymax]=[152,158], view_x=160, view_y=152, view_size_id=0, view_size=16
     * int cd = view_sizes_[sid];
    int left   = ((x-cd/2)/(cd/2))*(cd/2);
    int right  = ((x+cd/2)/(cd/2) +1)*(cd/2);
    int bottom = ((y-cd/2)/(cd/2))*(cd/2);
    int top    = ((y+cd/2)/(cd/2) +1)*(cd/2);
    if ( ((top-bottom)>cd || (right-left)>cd) && sid < view_sizes_.size()-1) {
        cd = view_sizes_[sid+1];
        left   = ((x-cd/2)/(cd/2))*(cd/2);
        right  = ((x+cd/2)/(cd/2) +1)*(cd/2);
        bottom = ((y-cd/2)/(cd/2))*(cd/2);
        top    = ((y+cd/2)/(cd/2) +1)*(cd/2);
    }
    int view_x = (left+right)/2;
    int view_y = (bottom+top)/2;
    view_size_ = cd;*/
    //view_size_ = cr*2;
    
//     double w = pdf.info.resolution*pdf.info.width;
//     double h = pdf.info.resolution*pdf.info.height;
//     ///#limits=(w/2, w, h/2, h)
//     ///#c = ((limits[0]+limits[1])*pdf.info.resolution/2, (limits[2]+limits[3])*pdf.info.resolution/2)
//     const int min_number_of_vertices_in_interest_area = 20;
//     double max_area_size = pdf.info.resolution*std::max({xmax-xmin, ymax-ymin, min_number_of_vertices_in_interest_area});
//     ROS_INFO("%s: max_area_size=%f", getName().c_str(), max_area_size);
    double cam_distance = (1.5  -  0.35*(view_sizes_[view_size_id_]-20)/250.0)*view_sizes_[view_size_id_]*pdf.info.resolution;//2+2*(int(max_area_size*0.2)/2);
    double cam_x = pdf.info.resolution*view_x;
    double cam_y = pdf.info.resolution*view_y;
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

void InferenceUnit::updatePdfAndPublish() {
    if (isNode) {
        if (check_sync_ && (state == INFERRED || pdf.header.seq - 1 != cmd_detected->header.seq - 1 || pdf.header.seq - 1 != map_divided->header.seq))
        {
            ROS_FATAL("%s: SYNCHRONIZATION BROKEN! state%s=INFERRED (must not be equal to INFERRED), pdf.seq==%d, cmd_detected.seq==%d, map_divided.seq==%d.",
                      getName().c_str(), (state == INFERRED ? "=" : "!"), pdf.header.seq, cmd_detected->header.seq, map_divided->header.seq);
            ros::shutdown();
            exit(1);
        }
    }
    new_pdf_ = false;
    updatePdf();
    denullifyPdf(pdf.data);
    updateInferenceState();
    pubPdf();
}

void InferenceUnit::updateInferenceState() {
    if (state==INFERRING) {
        ROS_WARN("%s: state INFERRING, thresh_high=%f, max_prob=%f", getName().c_str(), thresh_high, max_prob);
        if (max_prob >= thresh_high) {
            state = INFERRED;
            pubPositionInferred(max_prob_k);
            ROS_WARN("%s: state INFERRING -> INFERRED, pdf NOT published, /pose_inferred published max_prob=%f", getName().c_str(), max_prob);
            return;
        }
    } else { //state == INFERRING_NEW:
        ROS_WARN("%s: state INFERRING_NEW, thresh_high=%f, max_prob=%f", getName().c_str(), thresh_high, max_prob);
        if (max_prob <= thresh_low) {
            state = INFERRING;
            ROS_INFO("%s: state INFERRING_NEW -> INFERRING, max_prob=%f", getName().c_str(), max_prob);
        }
    }
}

void InferenceUnit::pubPdf() {
    if (interest_area_thresh_ > 0.0)
        publishViewTf();
    pdf.header.stamp = ros::Time::now();
    pub_pdf.publish(pdf);
    //normalizePdf();
    ros::spinOnce();
    ROS_INFO("%s: =========== published pdf (SEQ=%d), max_prob=%f", getName().c_str(), pdf.header.seq, max_prob);
    pdf.header.seq++;
}

void InferenceUnit::pubPositionInferred(int k) {
    int y = k / pdf.info.width;
    int x = k % pdf.info.width;
    pose_inferred.header.frame_id = "/map";
    SynchronizableNode::updatePose(pose_inferred, x, y, pdf.info.resolution);
    pub_position_inf.publish(pose_inferred);
    ROS_INFO("%s: published /pose_inferred, vertex=(%d,%d), pose=(%f,%f)", getName().c_str(), x, y, pose_inferred.pose.position.x, pose_inferred.pose.position.y);
}

// Normalize pdf to show in rviz
void InferenceUnit::normalizePdf() {
    double max_prob = 0;
    double min_prob = 1;
    long int imax = 0;
    long int imin = 0;
    for (std::size_t i=0;i < pdf.data.size();i++) {
        if (pdf.data[i] > max_prob) {
            max_prob = pdf.data[i];
            imax = i;
        }
        if (pdf.data[i] < min_prob && pdf.data[i] > 0) {
            min_prob = pdf.data[i];
            imin = i;
        }
    }
    for (std::size_t i=0;i < pdf.data.size();i++) {
        if(max_prob == min_prob) {
            ROS_WARN("MAX==MIN");
            return;
        }
        norm_pdf.data[i] = (100 - 0) / (max_prob) * pdf.data[i];
    }
}

void InferenceUnit::orientationInfCallback(std::vector<int>& unit_color, CommandConstPtr ptr_cmd) {
    cmd_detected = ptr_cmd;
    orientation_divided = unit_color;
    updateOrientationPdfAndPublish();
}

void InferenceUnit::setUniformOrientationPdf() {
    float uniform_prob = 1.0 / opdf.data.size(); 
    for (int i = 0;i < opdf.data.size();i++) {
        opdf.data[i] = uniform_prob;
    }
    ROS_WARN("uniform orientation prob:%f",uniform_prob);
}

void InferenceUnit::updateOrientationPdfAndPublish() {
    //calculate priors
    std::fill(priors.begin(),priors.end(),0.0);
    for (int k=0;k < opdf.data.size();k++) {
        if(orientation_divided[k] >= 0) {
            priors[orientation_divided[k]] += opdf.data[k];
        }
    }
    ROS_WARN("orientation priors: [%f,%f,%f,%f]", priors[0],priors[1],priors[2],priors[3]);

    //calculate coef
    calcUpdCoefs();

    max_oprob = 0.0;
    double total_oprob = 0.0;
    double p;
    for (int k=0; k<opdf.data.size(); k++) {
        p = opdf.data[k];
        if (p>=0) {
            opdf.data[k]=p*coefs[orientation_divided[k]];
            total_oprob += opdf.data[k];
            ///ROS_INFO("%s: updating pdf[%d]: %f->%f", getName().c_str(), k, p, pdf.data[k]);
            if (p >= max_oprob) {
                max_oprob   = p;
                max_oprob_k = k;
            }
        }
    }
    //smoothenPdf();

    denullifyPdf(opdf.data);

    //State??
    if (max_oprob > thresh_high) {
        state = ORIENTATION_INFERRED;
        orientation_inferred = max_oprob_k*orientation_resol*M_PI/180;
        ROS_WARN("Orientation Inferred: %f",orientation_inferred);
        pose_inferred.pose.orientation = tf::createQuaternionMsgFromYaw(orientation_inferred);
        pub_pose_inf.publish(pose_inferred);
    }
    //publish opdf
    opdf.header.stamp = ros::Time::now();
    pub_opdf.publish(opdf);
    ros::spinOnce();
    ROS_INFO("%s: =========== published opdf (SEQ=%d), max_oprob=%f", getName().c_str(), opdf.header.seq, max_oprob);
    opdf.header.seq++;
}