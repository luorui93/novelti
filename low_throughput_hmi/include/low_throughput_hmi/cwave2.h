#ifndef CWAVE2_H_
#define CWAVE2_H_

#include <vector>
#include <forward_list>
#include <low_throughput_hmi/map_if.h>


using namespace std;

//#define CHECK_MAP_BOUNDARIES 1


#ifdef CWAVE2_DEBUG
    #define CWAVE2_DBG(action, ...) glx.debugger.action(glx, ##__VA_ARGS__)
#else
    #define CWAVE2_DBG(...)
#endif

namespace cwave2 {

const int  MAP_CELL_EMPTY = -1;
const int  MAP_CELL_OCCUPIED = -10;
const bool IS_START=true;
const bool IS_END=false;
    
class OctPoint {
public:    
    char oct; //octant
    int x;
    int y;
    int F;
    bool reg;
    bool nbp;

    OctPoint(char oct0, int x0, int y0, int F0, bool reg0, bool nbp0);
};
    
    #ifdef CWAVE2_DEBUG
        class CWave2Debugger;
    #endif

class Point {
public:    
    int x;
    int y;
    
    Point(int x0, int y0);
    Point(OctPoint& p, Star& star)
}

class Boundary {
    struct ArcTip {
        int x;
        int y;
        int eps;
        bool offline;
        bool nbp;
        //char octant;
    };    
    
public:    
    char oct;
    int dx;
    int dy;
    
    int x; //cur value
    int y; //cur value
    int leps;  //used for line drawing (line epsilon"
    int eps;   //used for circle drawing eps = x^2+y^2-r^2
    
    ArcTip tip;
    
    Boundary(Point p, int eps);
    Boundary(OctPoint& op, int radius);
    void calcTip(int radius,   CWave2& cw);
    bool addPixel(CWave2& cw);
}


class Beam {
public:
    Boundary start;
    Boundary end;
    Beam::Beam();
};

class Star {
public:
    enum WalkState { 
        WALK_JUST_STARTED   = 0,
        WALK_WAIT_FOR_START = 1,
        WALK_WAIT_FOR_END   = 2,
        WALK_NEW_ARC_MAYBE  = 3,
    };    
    
    enum OriginState {
        ORIGIN_NOT_AT_START = 0,
        ORIGIN_AT_START     = 1,
        ORIGIN_CONNECTED    = 2
    };
    
    struct StarWalk {
        OctPoint     p;
        WalkState    state;    //unsigned char 
        bool         inclusive;         
        OriginState  origin;   //state of the origin (0deg)
        forward_list<Beam>::iterator curr;  //current beam
        forward_list<Beam>::iterator prev;  //previous beam
        OctPoint     prev_pt;  //previously visited cell coordinates     
        Boundary     tmp_end;
    };
    
    Point c;    //center coordinates
    int r;      //radius
    int dist;   //distance
    forward_list<Beam> beams;
    StarWalk w;
    
    Star(const Point& c0, int init_dist);
    bool empty();
    void addBeam(Beam beam);
    void deleteCurBeam();
    void grow(CWave2& cw);
    void arcWalk(CWave2& cw);
    bool visitPair(CWave2& cw);
    bool actOnPixel(int cost);
    void growCurBeam(CWave2& cw);
    void onBoundary(OctPoint& op, bool is_start, CWave2& cw);
}




class CWave2 {
    
    MapIf<int>& map;
    forward_list<Star> stars;
    forward_list<Star>::iterator ins;
    forward_list<Star>::iterator i;
    forward_list<Star>::iterator p;
    vector<Candidate> candidates_to_grow;
    vector<Candidate> candidates_to_add;
    int max_dist;
    #ifdef CWAVE2_DEBUG
        CWave2Debugger &debugger;
    #endif

    CWave2 (MapIf<int> &map
        #ifdef CWAVE2_DEBUG
            , CWaveDebugger &debugger
        #endif
    )

    calc(Point src, int max_dist);
    void bigBang(Point src, int max_dist);
    CWave2::setPixel(Point& p, int val);
    CWave2::getPixel(Point& p);
    void addCandidate(const Point& c);
    void starsGrow();
    void addToStars();
    int markPixel(Point p, bool is_nbp);
};

#ifdef CWAVE2_DEBUG
    class CWave2Debugger {
        public:
            virtual void highlight(Galaxy& glx, int x, int y, PixelType type)=0;
            virtual void star_print(Galaxy& glx, Star& s)=0;
            virtual void galaxy_print(Galaxy& glx)=0;
            virtual void map(Galaxy& glx)=0;
            virtual void highlight_boundary(Galaxy& glx, Star& star, Boundary& line)=0;
    };
#endif

}
#endif
