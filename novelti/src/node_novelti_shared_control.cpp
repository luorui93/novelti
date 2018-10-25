#include <novelti/novelti_shared_control.h>

using namespace novelti;


int main(int argc, char **argv) {
    ros::init(argc, argv, "novelti_shared_control");
    /*ros::NodeHandle n("~");

    std::string divMethod = "htile";
    std::string posMethod = "cog2lopt";
    n.getParam("div/method", divMethod);   
    n.getParam("pos/method", posMethod);*/

    NoveltiSharedControl nsc;
}