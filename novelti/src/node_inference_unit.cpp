//#define DEBUG_INFERENCE 1

/*
       subs                                            pubs
                   +----------------------------+
                   |                            |
/cmd_detected ---> |                            | ---> /pdf
 /map_divided ---> |      inference_unit        |
        /task ---> |                            |
                   |                            |
                   +----------------------------+
                         ^                ^
                         |                |
                     srv: start         srv: task
                         req:  scene        req:  "new_goal"
                         resp: -            resp: -

----------------------------------------------------------------------------------------

                                   +-----------+
           max_prob <= THRESH_LOW  |           | max_prob >= THRESH_HIGH
         +------------------------>| INFERRING +--------------------------+
         |                         |           |                          |
         |                         +-----------+                          |
         |                                                                v
 +----------------+                                               +----------------+
 |                |              task service req: "new_goal"     |                |
 | INFERRING_NEW  |<----------------------------------------------+    INFERRED    |
 |                |                                               |                |
 +----------------+                                               +----------------+
*/

#include <novelti/inference_unit.h>

using namespace novelti;

class InferenceUnitNode : public SynchronizableNode, public InferenceUnit {
public:
    InferenceUnitNode() {}
    void start(novelti::StartExperiment::Request & req) {
        startExp(req);
    }
    
    void stop() {
        stopExp();
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "inference_unit");
    ros::NodeHandle n("~");
    InferenceUnitNode iu;
    iu.run();
}
