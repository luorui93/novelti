#ifndef CWAVE2_H_
#define CWAVE2_H_

#include <vector>
#include <forward_list>
#include <CompoundMap.h>

#ifdef MULTITHREAD
    #include <thread>
    //#include <semaphore.h>
    //#include <sys/types.h>
    //#include <sys/syscall.h>
    //#define gettid() syscall(SYS_gettid)
    #include <atomic>
#endif


using namespace std;


#ifdef CWAVE2_DEBUG
    #include <CWave2Debugger.h>
#else
    #define CWAVE2_DBG(...)
#endif

namespace cwave {

    enum PairState {
        XO=0,
        OX=1,        
        OO=2,
        O_=3,
        X_=4,
        XX=5,
        vX=6,
        v_=7,
        vO=8,
        Ov=9,
        vv=10,
    };
    
class CWave2;
class Star;
class ThreadPool;


const bool ODD=true;
const bool EVEN=false;


class OctPoint {
public:
    char oct; //octant
    int x;
    int y;
    //int F;
    int eps;
    
    OctPoint(){};
    OctPoint(char oct0, int x0, int y0, int eps);
    
    void xpp(bool odd_octant);
    void xmm(bool odd_octant);
    void ypp(bool odd_octant);
    void ymm(bool odd_octant);
    
    bool isDiag(bool odd_octant);
    bool isDiagNBP(bool odd_octant, int r);
    int geteps();
    int nbpeps();
};

        
class Point {
public:    
    int x;
    int y;
    
    Point() {};
    Point(int x0, int y0);
};

class Boundary {
    struct ArcTip {
        int x;
        int y;
        int eps;
        bool offline;
        bool nbp;
        bool was_incremented;
    };    
    
public:
    
    enum BoundaryType {
        BOUNDARY_TYPE_A,
        BOUNDARY_TYPE_B
    };
    
    char oct;
    int dx;
    int dy;
    
    int x; //cur value
    int y; //cur value
    int leps;  //used for line drawing (line epsilon"
    int eps;   //used for circle drawing eps = x^2+y^2-r^2
    BoundaryType type;
    bool was_incremented;
    ArcTip tip;
    bool overlap;
    
    Boundary(){};
    Boundary(char octant);
    //Boundary(Point p, int eps);
    //Boundary(Boundary& b, bool is_start, CWave2& cw);
    Boundary(char oct0, int dx0, int dy0, int leps0, BoundaryType type, bool is_start, CWave2& cw, Star& s);
    bool update(OctPoint& op, PairState occup, int radius, bool is_start);
    void calcTip(int radius,   CWave2& cw, Star& s);
    bool addPixel(CWave2& cw, Star& s);
    bool toIncrement();
    bool wasIncremented();
};


class Beam {
public:
    Boundary start;
    Boundary end;
    Beam();
    Beam(Boundary s, Boundary e);
    //bool isEmpty();
};

#ifdef CWAVE2_FLOAT_STARS
    typedef double StarDist;
#else
    typedef int StarDist;   //distance
#endif

class Star {
public:
    Point c;    //center coordinates
    int r;      //radius
    #ifdef CWAVE2_FLOAT_STARS
        StarDist dist;
    #endif
    int idist;
    forward_list<Beam> beams;
    int id;
    
    Star() {};
    Star(const Point& c0, StarDist init_dist, const Beam& beam, bool two_beams);
    bool  empty();
    Point octToPixel(char oct, int x, int y);
    Point octToPoint(OctPoint& p);
    Point octToPoint(char oct, int x, int y);
    void  grow(CWave2& cw);
};


class Walker {
public:
    
    enum WalkState { 
        WALK_JUST_STARTED    = 0,
        WALK_WAIT_FOR_START  = 1,
        WALK_WAIT_FOR_END    = 2,
        WALK_NEW_ARC_MAYBE   = 3,
        WALK_WAIT_FOR_END_MB = 4,
    };    
    
    enum Action {
        ACTION_NONE         = 0,
        ACTION_NEW_START    = 1,
        ACTION_NEW_BEAM     = 2,
        ACTION_NEW_END      = 3,
    };
    
    Star&       star;
    CWave2&     cw;
    OctPoint    pt;
    OctPoint    pt_prev;  //previously visited cell coordinates  
    PairState        occup_prev;
    forward_list<Beam>::iterator beam;
    forward_list<Beam>::iterator beam_prev;  //previous beam

    WalkState   state;    //unsigned char 
    bool        inclusive;
    bool        to_mark_prev;
       
    Boundary    tmp_end;
    
    Walker(Star& s, CWave2& cw);
    void addBeam(Beam new_beam);
    void deleteCurBeam();
    
    bool preWalk();
    bool arcWalk(bool);
    void postWalk(bool);
    void visitPair(bool nbp, bool to_mark);
    void visitIndepNBP(bool);
    bool processPoint(PairState occup, bool to_mark);

    void act(Action a, PairState occup);
    void grow();
    void onBoundary(OctPoint& op, bool is_start);
    bool setPoint(OctPoint& pt, bool is_nbp, bool to_mark);
    bool isOverlap(char oct, int x, int y);
    bool isPixelFree(char oct, int x, int y);
    void addStar(char oct, int dx, int dy, int leps, Boundary::BoundaryType type, bool is_start, bool reducedDist);
    void addStarBefore(PairState occup);
    void addStarAfter(PairState occup);
    
    bool updateRegPoint(int x, int y);
    PairState visitRegPoint(bool no_mark);
};

#ifdef MULTITHREAD
    typedef atomic<int> iter_int;
#else
    typedef int iter_int;
#endif

    
#ifdef CWAVE2_PROC
    class CWave2Processor {
        public:
            /* onSetPointDist(...) is called whenever a vertex (point) value is updated
             *      cw:         CWave2 object
             *      op:         vertex relative coordinates w.r.t. to its star (for an NBP, these are coordinates of the paired regular vertex) 
             *      is_nbp:     wether the vertex is an NBP
             *      p:          vertex absolute coordinates
             *      old_dist:   vertex value before update (if old_dist==MAP_POINT_UNEXPLORED, then the vertex has not been assigned any distance before)
             *      new_dist:   vertex value after update
             */
            CWave2Processor() {};
            virtual void onInitSource(Point& p) {};
            virtual void onAddStar(Star& parent_star, Star& s) {};
            virtual void onSetPointDistance(Star& star, OctPoint& op, bool is_nbp, Point& p, int old_dist, int new_dist) {};
            virtual void onDistanceCorrection(Star& parent_star, Point& p, int old_dist, int new_dist) {};
//             virtual void onNextStar() {int star_id};
//             virtual void onNextBeam() {};
//             virtual void onBeamDeletion() {};
//             virtual void onBeamAddition() {};
//             virtual void onBeamStart() {};
//             virtual void onBeamEnd() {};
//             virtual void onRecursiveCall() {};
//             virtual void onRecursiveReturn() {};
    };
#endif
    
    

class CWave2 {
public:
    CompoundMap& map;
    forward_list<Star> stars;
    
    vector<Star> added[3];
    vector<Star> proc;
    iter_int a[3];
    iter_int pr;
    
    forward_list<Star>::iterator i;
    forward_list<Star>::iterator p;
    #ifdef CWAVE2_FLOAT_STARS
        forward_list<Star>::iterator insert[3];
    #endif
    int dist;
    int max_dist;
    #ifdef CWAVE2_DEBUG
        CWave2Debugger &debugger;
        CWave2 (CompoundMap &map, CWave2Debugger &debugger);
        static void calculateDistances(CompoundMap &map, Point src, CWave2Debugger &dbg, ThreadPool* pool);
    #else
        CWave2 (CompoundMap &map);
        static void calculateDistances(CompoundMap &map, Point src, ThreadPool* pool);
    #endif
    #ifdef CWAVE2_PROC
        CWave2Processor* processor;
        void setProcessor(CWave2Processor* proc) { processor = proc; };
    #endif
    iter_int nstars;
    #ifdef MULTITHREAD
        ThreadPool* threadPool;
        
    #endif
    bool updateMapPoint(int x, int y, int new_dist);
    void resetStars();
    bool postProcStars();
    void addStar(Star s, Star& parent_star);
    Star* getStar();
    void starsGrow();

    void calc(Point src);
    void calc(Point src, int max_dist);
    void bigBang(Point src, int max_dist);
    
    
};

const int START = 0;
const int STOP = 1;
const int FINISHED = 2;

class ThreadPool {
public:
    #ifdef MULTITHREAD
    vector<atomic<int>> statuses;
    vector<thread> threads;
    #endif
    CWave2* cwave;    
    ThreadPool(int nthreads);
    ~ThreadPool();
protected:
    static void worker(ThreadPool* pool, int tid);
};


}
#endif
