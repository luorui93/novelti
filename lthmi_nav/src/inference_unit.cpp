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
    node.param<float>("eps", eps, 1.0e-12);
    
    node.getParam("interface_matrix", interface_matrix);
    int n_cmds = (int)floor(sqrt(interface_matrix.size()));
    if (interface_matrix.size()==0 && n_cmds*n_cmds==interface_matrix.size())
        throw ros::Exception("ERROR: interface_matrix parameter should be specified, have length>0, and its length has to be a square of an integer number (number of commands)");
    priors = std::vector<double>(n_cmds, 0.0);
    posteriors = std::vector<double>(n_cmds, 0.0);
    coefs = std::vector<double>(n_cmds, 0.0);
    srv_new_goal = node.advertiseService("new_goal", &InferenceUnit::srvNewGoal, this);
}

bool InferenceUnit::srvNewGoal(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp) {
    state = INFERRING_NEW;
    ROS_INFO("%s: new_goal service request received. State INFERRING -> INFERRING_NEW", getName().c_str());
    return true;
}

void InferenceUnit::stop() {
    pub_pdf.shutdown();
    pub_pose_inf.shutdown();
    sub_map_div.shutdown();
    sub_cmd.shutdown();
}

void InferenceUnit::start(lthmi_nav::StartExperiment::Request& req) {
    state = INFERRING;
    
    pdf.header.frame_id = "/map";
    pdf.info.width = req.map.info.width+1;
    pdf.info.height = req.map.info.height+1;
    pdf.info.resolution = req.map.info.resolution;
    pdf.data = std::vector<float>(pdf.info.width*pdf.info.height, PDF_UNREACHABLE);
    int k;
    //determine reachable vertices
    long int total_vx = 0;
    for (int x=1; x<req.map.info.width; x++) {
        for (int y=1; y<req.map.info.height; y++) {
            k = x + y*req.map.info.width;
            if (req.map.data[k]==0 || req.map.data[k+1]==0 || req.map.data[k+req.map.info.width]==0 || req.map.data[k+1+req.map.info.width]==0) {
                pdf.data[x + y*pdf.info.width] = 0.0;
                total_vx++;
            }
        }
    }
    
    //set uniform pdf over reachable vertices
    float vx_prob = 1.0/total_vx;
    for (int k=pdf.data.size()-1;k>=0; k--)
        if (pdf.data[k] != PDF_UNREACHABLE)
            pdf.data[k] = vx_prob;
    
    pub_pdf      = node.advertise<FloatMap>("/pdf", 1, false); //not latched
    pub_pose_inf = node.advertise<geometry_msgs::PoseStamped>("/pose_inferred", 1, false); //not latched
    sub_map_div  = node.subscribe("/map_divided", 1, &InferenceUnit::mapDivCallback, this);
    sub_cmd      = node.subscribe("/cmd_detected", 1, &InferenceUnit::cmdCallback, this);
}

void InferenceUnit::denullifyPdf() { //replaces small probs (<eps) with eps, and normalizes
    double p, sc=0.0, sa=0.0;
    for (int k=pdf.data.size()-1; k>=0; k--) { //replace small probs (<eps) with eps 
        p = pdf.data[k];
        if (p != PDF_UNREACHABLE && p < eps) {
            sc += eps;
            sa += p;
        }
    }
    double c=(1.0-sc)/(1.0-sa);
    for (int k=pdf.data.size()-1; k>=0; k--) { //normalize
        p = pdf.data[k];
        if (p != PDF_UNREACHABLE) {
            pdf.data[k] = p<eps ? eps : c*p;
        }
    }
}

    
void InferenceUnit::calcPriors() {
    std::fill(priors.begin(), priors.end(), 0.0); //priors = 0-vector
    for (int k=0; k<map_divided->data.size(); k++)
        if (map_divided->data[k]>=0)
            priors[map_divided->data[k]] += pdf.data[k];
    ROS_INFO("%s: priors from map_divided: [%f, %f, %f, %f]", getName().c_str(), priors[0], priors[1], priors[2], priors[3]);
}

void InferenceUnit::calcUpdCoefs(int cmd_detected) {
    std::fill(posteriors.begin(), posteriors.end(), 0.0); //posteriors = 0-vector
    double total = 0.0;
    for (int k=0; k<n_cmds; k++) {
        posteriors[k] += interface_matrix[cmd_detected + k*n_cmds] * priors[k]; //TODO double check
        total += posteriors[k];
        //rospy.loginfo("inference_module: priors = %s" % str(priors))
        //rospy.loginfo("inference_module: interface_matrix[k][detected] = %s, k=%d, detected=%d", str(interface_matrix[k][detected]), k, detected)
    }
    ROS_INFO("%s: calculated posteriors before normalization: [%f, %f, %f, %f]", getName().c_str(), posteriors[0], posteriors[1], posteriors[2], posteriors[3]);
    
    //normalize posteriors (not sure if needed)
    for (int k=0; k<n_cmds; k++)
        posteriors[k] = posteriors[k]/total;
    ROS_INFO("%s: calculated posteriors after normalization: [%f, %f, %f, %f]", getName().c_str(), posteriors[0], posteriors[1], posteriors[2], posteriors[3]);
    
    //caluclate update coefficients
    for (int k=0; k<n_cmds; k++) 
         coefs[k] = posteriors[k]/priors[k] ? priors[k] != 0.0 : 0.0;
}


void InferenceUnit::mapDivCallback(lthmi_nav::IntMapConstPtr msg){
    ROS_INFO("%s: received map_divided", getName().c_str());
    if (state==INFERRED)
        return;
    map_divided = msg;
    calcPriors();
}

void InferenceUnit::updatePdf(int cmd_detected) {
    calcUpdCoefs(cmd_detected);
    max_prob = 0.0;
    double p;
    for (int k=0; k<pdf.data.size(); k++) {
        p = pdf.data[k];
        if (p>=0) {
            pdf.data[k]=p*coefs[map_divided->data[k]];
            ROS_INFO("%s: updating pdf[%d]: %f->%f", getName().c_str(), k, p, pdf.data[k]);
            if (p >= max_prob) {
                max_prob   = p;
                max_prob_k = k;
            }
        }
    }
}

void InferenceUnit::cmdCallback(CommandConstPtr msg){
    ROS_INFO("%s: received cmd_detected = %d", getName().c_str(), msg->cmd);
    switch (state) {
        case INFERRED:
            ROS_INFO("%s: state==INFERRED, ignoring", getName().c_str());
            return;
        case INFERRING:
            updatePdf(msg->cmd);
            if (max_prob >= thresh_high) {
                state = INFERRED;
                pubPoseInferred(max_prob_k);
                ROS_INFO("%s: state INFERRING -> INFERRED, pdf NOT published. max_prob=%f", getName().c_str(), max_prob);
                return;
            }
            break;
        case INFERRING_NEW:
            updatePdf(msg->cmd);
            if (max_prob <= thresh_low) {
                state = INFERRING;
                ROS_INFO("%s: state INFERRING_NEW -> INFERRING, max_prob=%f", getName().c_str(), max_prob);
            }
    }
    denullifyPdf();
    pubPdf();
}

void InferenceUnit::pubPdf() {
    pdf.header.stamp = ros::Time::now();
    //pdf.info.origin.position.z = 0.5;
    pub_pdf.publish(pdf);
    ROS_INFO("%s: published updated pdf, max_prob=%f", getName().c_str(), max_prob);
}

void InferenceUnit::pubPoseInferred(int k) {
    int y = k / map_divided->info.width;
    int x = k % map_divided->info.width;
    geometry_msgs::PoseStamped pose = Vertex::toPose(x, y, pdf.info.resolution);
    pose.header.frame_id = "/map";
    pub_pose_inf.publish(pose);
    ROS_INFO("%s: published /pose_inferred, vertex=(%d,%d), pose=(%f,%f)", getName().c_str(), x, y, pose.pose.position.x, pose.pose.position.y);
}
