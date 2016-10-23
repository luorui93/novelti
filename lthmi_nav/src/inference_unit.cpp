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

#include <lthmi_nav/inference_unit.h>


using namespace lthmi_nav;

InferenceUnit::InferenceUnit() :
        SynchronizableNode()
{
    node.param<float>("thresh_high", thresh_high, 0.98);
    node.param<float>("thresh_low", thresh_low, 0.5);
    node.param<double>("eps", eps, 1.0e-12);
    
    node.getParam("interface_matrix", interface_matrix);
    n_cmds = (int)floor(sqrt(interface_matrix.size()));
    if (interface_matrix.size()==0 && n_cmds*n_cmds==interface_matrix.size())
        throw ros::Exception("ERROR: interface_matrix parameter should be specified, have length>0, and its length has to be a square of an integer number (number of commands)");
//      for (int x=0; x<4; x++)
//          for (int y=0; y<4; y++){
//              double q = interface_matrix[x + n_cmds*y];
//              ROS_INFO(">>>>>>>>>>>>>>> %f", q);}
    priors     = std::vector<double>(n_cmds, 0.0);
    posteriors = std::vector<double>(n_cmds, 0.0);
    coefs      = std::vector<double>(n_cmds, 0.0);
    srv_new_goal = node.advertiseService("new_goal", &InferenceUnit::srvNewGoal, this);
}

bool InferenceUnit::srvNewGoal(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp) {
    state = INFERRING_NEW;
        ROS_INFO("%s: new_goal service request received. State INFERRING -> INFERRING_NEW", getName().c_str());
    denullifyPdf();//??
    pubPdf();
    return true;
}

void InferenceUnit::stop() {
    ROS_INFO("++++++++++++++++ 1");
    sub_map_div.shutdown();
    ROS_INFO("++++++++++++++++ 2");
    sub_cmd.shutdown();
    ROS_INFO("++++++++++++++++ 3");
    pub_pdf.shutdown();
    ROS_INFO("++++++++++++++++ 4");
    pub_pose_inf.shutdown();
    ROS_INFO("++++++++++++++++ 5");

}

void InferenceUnit::start(lthmi_nav::StartExperiment::Request& req) {
    state = INFERRING;
    fast_state = RCVD_NONE;
    ROS_INFO("**************** 1");
    pdf = FloatMap();
    pdf.header.frame_id = "/map";
    pdf.info.width = req.map.info.width+1;
    pdf.info.height = req.map.info.height+1;
    pdf.info.resolution = req.map.info.resolution;
    pdf.info.origin.position.x = -0.5*req.map.info.resolution;
    pdf.info.origin.position.y = -0.5*req.map.info.resolution;
    pdf.data = std::vector<float>(pdf.info.width*pdf.info.height, PDF_UNREACHABLE);
    int k;
    ROS_INFO("**************** 2");
    //determine reachable vertices
    long int total_vx = 0;
    for (int x=0; x<req.map.info.width-1; x++) {
        for (int y=0; y<req.map.info.height-1; y++) {
            k = x + y*req.map.info.width;
            if (req.map.data[k]==0 || req.map.data[k+1]==0 || req.map.data[k+req.map.info.width]==0 || req.map.data[k+1+req.map.info.width]==0) {
                pdf.data[x+1 + (y+1)*pdf.info.width] = 0.0;
                total_vx++;
            }
        }
    }
    ROS_INFO("**************** 3");
    //set uniform pdf over reachable vertices
    float vx_prob = 1.0/total_vx;
    for (int k=pdf.data.size()-1;k>=0; k--)
        if (pdf.data[k] != PDF_UNREACHABLE)
            pdf.data[k] = vx_prob;
    ROS_INFO("**************** 4");
    pub_pdf      = node.advertise<FloatMap>("/pdf", 1, true); //not latched
    ROS_INFO("**************** 4-1");
    pub_pose_inf = node.advertise<geometry_msgs::PoseStamped>("/pose_inferred", 1, false); //not latched
    ROS_INFO("**************** 4-2");
    sub_map_div  = node.subscribe("/map_divided", 1, &InferenceUnit::mapDivCallback, this);
    ROS_INFO("**************** 4-3");
    sub_cmd      = node.subscribe("/cmd_detected", 1, &InferenceUnit::cmdCallback, this);
    ROS_INFO("**************** 5");
}

void InferenceUnit::denullifyPdf() { //replaces small probs (<eps) with eps, and normalizes
    double p, sc=0.0, sa=0.0;
    //ROS_INFO("++++++++++++++++ eps=%f", eps);
    for (int k=pdf.data.size()-1; k>=0; k--) { //replace small probs (<eps) with eps 
        p = pdf.data[k];
        if (p != PDF_UNREACHABLE && p < eps) {
            sc += eps;
            sa += p;
        }
    }
    double c=(1.0-sc)/(1.0-sa);
    //ROS_INFO("++++++++++++++++ c=%f", c);
    for (int k=pdf.data.size()-1; k>=0; k--) { //normalize
        p = pdf.data[k];
        if (p != PDF_UNREACHABLE) {
            pdf.data[k] = p<eps ? eps : c*p;
        }
    }
    
    double min_prob = 1.0;
    for (int k=pdf.data.size()-1; k>=0; k--) //find min prob
        if (pdf.data[k]>=0.0 && pdf.data[k]<min_prob)
            min_prob = pdf.data[k];
    //ROS_INFO("++++++++++++++++ min_prob=%f", min_prob*1000000);
}

    
void InferenceUnit::calcPriors() {
    ROS_INFO("################# 1");
    std::fill(priors.begin(), priors.end(), 0.0); //priors = 0-vector
    ROS_INFO("################# 2");
    for (int k=0; k<map_divided->data.size(); k++)
        if (pdf.data[k]>=0) {
            //ROS_INFO("################# reg=%d", map_divided->data[k]);
           // ROS_INFO("################# p=%f", pdf.data[k]);
            priors[map_divided->data[k]] += pdf.data[k];
        }
    ROS_INFO("%s: priors from map_divided: [%f, %f, %f, %f]", getName().c_str(), priors[0], priors[1], priors[2], priors[3]);
}

void InferenceUnit::calcUpdCoefs() {
    std::fill(posteriors.begin(), posteriors.end(), 0.0); //posteriors = 0-vector
    double total = 0.0;
    for (int k=0; k<n_cmds; k++) {
        //ROS_INFO(">>>>>>>>>>>>>>> mx=%f, pr=%f", interface_matrix[cmd_detected->cmd + k*n_cmds], priors[k]);
        posteriors[k] += interface_matrix[cmd_detected->cmd + k*n_cmds] * priors[k]; //TODO double check
        total += posteriors[k];
    }
    ROS_INFO("%s: calculated posteriors before normalization: [%f, %f, %f, %f]", getName().c_str(), posteriors[0], posteriors[1], posteriors[2], posteriors[3]);
    
    //normalize posteriors (not sure if needed)
    for (int k=0; k<n_cmds; k++)
        posteriors[k] = posteriors[k]/total;
    ROS_INFO("%s: calculated posteriors after normalization: [%f, %f, %f, %f]", getName().c_str(), posteriors[0], posteriors[1], posteriors[2], posteriors[3]);
    
    //caluclate update coefficients
    for (int k=0; k<n_cmds; k++) 
         coefs[k] = priors[k] != 0.0 ? posteriors[k]/priors[k] : 0.0;
    ROS_INFO("%s: coefs: [%f, %f, %f, %f]", getName().c_str(), coefs[0], coefs[1], coefs[2], coefs[3]);
}


void InferenceUnit::mapDivCallback(lthmi_nav::IntMapConstPtr msg){
    ROS_INFO("%s: received map_divided (SEQ=%d)", getName().c_str(), msg->header.seq);
    map_divided = msg;
    calcPriors();
    if (fast_state==RCVD_CMD) {
        fast_state = RCVD_NONE;
        updatePdfAndPublish();
    } else {
        fast_state = RCVD_MAPDIV;
        ROS_INFO("%s: fast_state := RCVD_MAPDIV", getName().c_str());
    }
}
void InferenceUnit::cmdCallback(CommandConstPtr msg){
    ROS_INFO("%s: received cmd_detected = %d (SEQ=%d)", getName().c_str(), msg->cmd, msg->header.seq);
    cmd_detected = msg;
    if (fast_state==RCVD_MAPDIV) {
        fast_state = RCVD_NONE;
        updatePdfAndPublish();
    } else {
        fast_state = RCVD_CMD;
        ROS_INFO("%s: fast_state := RCVD_CMD", getName().c_str());
    }
}

void InferenceUnit::updatePdf() {
    calcUpdCoefs();
    max_prob = 0.0;
    double total_prob = 0.0;
    double p;
    for (int k=0; k<pdf.data.size(); k++) {
        p = pdf.data[k];
        if (p>=0) {
            pdf.data[k]=p*coefs[map_divided->data[k]];
            total_prob += pdf.data[k];
            ///ROS_INFO("%s: updating pdf[%d]: %f->%f", getName().c_str(), k, p, pdf.data[k]);
            if (p >= max_prob) {
                max_prob   = p;
                max_prob_k = k;
            }
        }
    }
    ROS_INFO("%s: total prob=%f", getName().c_str(), total_prob);
}

void InferenceUnit::updatePdfAndPublish() {
    if (state==INFERRED || pdf.header.seq-1 != cmd_detected->header.seq-1 || pdf.header.seq-1 != map_divided->header.seq) {
        ROS_FATAL("%s: SYNCHRONIZATION BROKEN! state%s=INFERRED (must not be equal to INFERRED), pdf.seq==%d, cmd_detected.seq==%d, map_divided.seq==%d.",
                 getName().c_str(), (state==INFERRED ? "=" : "!"), pdf.header.seq, cmd_detected->header.seq, map_divided->header.seq);
        ros::shutdown();
        exit(1);
    }
    updatePdf();
    denullifyPdf();
    if (state==INFERRING) {
        ROS_INFO("%s: state INFERRING, thresh_high=%f", getName().c_str(), thresh_high);
        if (max_prob >= thresh_high) {
            state = INFERRED;
            pubPoseInferred(max_prob_k);
            ROS_INFO("%s: state INFERRING -> INFERRED, pdf NOT published, /pose_inferred published max_prob=%f", getName().c_str(), max_prob);
            return;
        }
    } else { //state == INFERRING_NEW:
        ROS_INFO("%s: state INFERRING_NEW", getName().c_str());
        if (max_prob <= thresh_low) {
            state = INFERRING;
            ROS_INFO("%s: state INFERRING_NEW -> INFERRING, max_prob=%f", getName().c_str(), max_prob);
        }
    }
    pubPdf();
}

void InferenceUnit::pubPdf() {
    pdf.header.stamp = ros::Time::now();
    //pdf.info.origin.position.z = 0.5;
    //ROS_INFO("%s: before published pdf (SEQ=%d)", getName().c_str(), pdf.header.seq);
    //pub_pdf.publish(pdf);
    //ros::spinOnce();
    //ROS_INFO("%s: before published 1 pdf (SEQ=%d)", getName().c_str(), pdf.header.seq);
    pub_pdf.publish(pdf);
    ros::spinOnce();
    ROS_INFO("%s: =========== published pdf (SEQ=%d), max_prob=%f", getName().c_str(), pdf.header.seq, max_prob);
    pdf.header.seq++;
}

void InferenceUnit::pubPoseInferred(int k) {
    int y = k / map_divided->info.width;
    int x = k % map_divided->info.width;
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "/map";
    updatePose(pose, x, y);
    pub_pose_inf.publish(pose);
    ROS_INFO("%s: published /pose_inferred, vertex=(%d,%d), pose=(%f,%f)", getName().c_str(), x, y, pose.pose.position.x, pose.pose.position.y);
}
