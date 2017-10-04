//#define CWAVE2_DEBUG 1


#include <stdio.h>
#include <cstring>
#include <CompoundMap.h>
#include <CWave2.h>
#include <CWave2DebuggerSDL.cpp>
#include <chrono>
#include <thread>

static const char *doc="USAGE:\n"
"\n"
"Calculate distance from a source vertex on a grid map to a set of vertices.\n"
"    $ cat map_file.map | ./cwave_cmdline   one2many   source_x source_y    x1 y1    x2 y2    x3 y3    ....\n"
"    distance_from_source_to_point1\n"
"    distance_from_source_to_point2\n"
"    distance_from_source_to_point3\n"
"    ...\n"
"Print this help:\n"
"   $ %s help\n";

void incorrect_usage(const char* msg, char* exec_file) {
    fprintf(stderr, "Incorrect command line usage.\n");
    fprintf(stderr, msg);
    fprintf(stderr, "\nTo see help, issue:\n$ %s help", exec_file);
    exit(1);
}

int pose_to_vertex_tolerance = 4;

double getDistance(cwave::CompoundMap& cmap, int& cx,int& cy) {
    int dx, dy, xmin=cx, ymin=cy;
    double d, dmin = std::numeric_limits<double>::max();
    if (cmap.getPoint(cx,cy) == cwave::MAP_POINT_UNEXPLORED) {
        //ROS_WARN("SEARCH for accessible vertex center=(%d,%d)", cx,cy);
        for (int x=max(0,cx-pose_to_vertex_tolerance); x<=min(cmap.width()-1, cx+pose_to_vertex_tolerance); x++) {
            for (int y=max(0,cy-pose_to_vertex_tolerance); y<=min(cmap.height()-1, cy+pose_to_vertex_tolerance); y++) {
                //ROS_WARN("try: (%d,%d)", x,y);
                if (cmap.getPoint(x,y) != cwave::MAP_POINT_UNEXPLORED) {
                    dx = cx-x;
                    dy = cy-y;
                    d = dx*dx + dy*dy;
                    //ROS_WARN("accessible! d=%f, dmin=%f", d, dmin);
                    if (d<dmin) {
                        dmin = d;
                        xmin = x;
                        ymin = y;
                    }
                }
            }
        }
        if (dmin == std::numeric_limits<double>::max())
            exit(77);//("%s: could not find an accessible vertex nearby current pose (x,y)=(%f,%f), checked %d vertices in all directions", getName().c_str(), px, py, pose_to_vertex_tolerance);
    }
    return cmap.getExactDist(xmin, ymin);
}

int main(int argc, char **argv) {
    FILE *input = stdin;
    if (argc<2)
        incorrect_usage("At least one command line argument has to be provided", argv[0]);
    
    if (strcmp(argv[1], "help")==0) {
        printf(doc, argv[0]);
    } else if (strcmp(argv[1], "one2many")==0) {
        if (argc<4) 
            incorrect_usage("For one2many command, at least two additional arguments have to be provided.", argv[0]);
        if (argc%2 != 0) 
            incorrect_usage("For one2many command, the number of additional arguments has to be even.", argv[0]);
        int src_x = atoi(argv[2]);
        int src_y = atoi(argv[3]);
        //printf("src=(%d,%d)\n", src_x, src_y);
        cwave::CompoundMap cmap(stdin);
        //printf("w,h = %d,%d\n", cmap.width(), cmap.height());
        // std::this_thread::sleep_for(std::chrono::seconds(5));
        #ifdef CWAVE2_DEBUG
            cwave::CWave2DebuggerSDL dbg(cmap, 0.95);
            printf("cmap(87,200)==%d\n", cmap.isPixelOccupied(10,16));
            cwave::CWave2 cw(cmap, dbg);
        #else
            cwave::CWave2 cw(cmap);
        #endif
        //std::this_thread::sleep_for(std::chrono::seconds(5));
        cwave::CWave2Processor dummy;
        cw.setProcessor(&dummy);
        cw.calc(cwave::Point(src_x,src_y));
       
        int x,y,d;
        for (int k=4; k<argc; k+=2 ) {
            x = atoi(argv[k]);
            y = atoi(argv[k+1]);
            printf("%f\n", getDistance(cmap, x,y));
        }
    } else {
        incorrect_usage("Unknown command", argv[0]);
    }
}
