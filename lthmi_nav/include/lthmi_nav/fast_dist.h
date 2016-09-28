#ifndef CWAVE_H_
#define CWAVE_H_

#include <vector>
#include <forward_list>
#include <lthmi_nav/map_if.h>


using namespace std;

//#define TRACK_MAP 1
//#define PDF_STAT 1
#define CHECK_MAP_BOUNDARIES 1


#ifdef CIRC_DIV
    #define PDF_STAT 1
#endif

#ifdef MEAN_DIST
    #define PDF_STAT 1
#endif

#ifdef PDF_STAT
        #define if_mean_dist(smth) smth
#else
        #define if_mean_dist(smth)
#endif


#ifdef TRACK_MAP
        #define if_track_then(smth) smth
#else
        #define if_track_then(smth)
#endif

#ifdef CWAVE_DEBUG
    #define CWAVE_DBG(action, ...) glx.debugger.action(glx, ##__VA_ARGS__)
#else
    #define CWAVE_DBG(...)
#endif

namespace lthmi_nav {
    
    #define MAP_CELL_EMPTY -1
    #define MAP_CELL_OCCUPIED -10
    #define MAP_CELL_OUT_OF_BOUNDARY -20
    
    #ifdef CWAVE_DEBUG
        class CWaveDebugger;
    #endif
    
    struct OctPoint {
        char oct; //octant
        int x;
        int y;
        int F;
        bool with_nbp;
    };
    
    struct ArcTip {
        int x;
        int y;
        int eps;
        bool offline;
        bool nbp;
        //char octant;
    };
    
    struct Boundary {
        char octant;
        int dx;
        int dy;
        
        int x;
        int y;
        int leps;
        int eps;        
        
        ArcTip tip;
    };
    


    struct Beam {
        Boundary start;
        Boundary end;
    };
    
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
        WalkState    state;    //unsigned char 
        bool         inclusive;         
        OriginState  origin;   //state of the origin (0deg)
        forward_list<Beam>::iterator curr;  //current beam
        forward_list<Beam>::iterator prev;  //previous beam
        OctPoint     prev_pt;  //previously visited cell coordinates     
        Boundary     tmp_end;
    };
    
    struct Star {
        int x;
        int y;
        int r;      //radius
        int dist;   //distance
        forward_list<Beam> beams;
        StarWalk walk;
        #ifdef TRACK_MAP
            int id;
        #endif
    };
    
    struct Candidate {
        int x;
        int y;
        #ifdef TRACK_MAP
            int parent_id;
        #endif
    };
    
    struct Point2D {
        int x;
        int y;
    };
    
    struct Galaxy {
        MapIf<int>& map;
        forward_list<Star> stars;
        forward_list<Star>::iterator ins;
        forward_list<Star>::iterator i;
        forward_list<Star>::iterator p;
        vector<Candidate> candidates_to_grow;
        vector<Candidate> candidates_to_add;
        int dist;
        #ifdef TRACK_MAP
            vector<Point2D> track_stars;
            int cur_star_id;
            MapIf<int>& track_map;
        #endif
        int max_dist;
        #ifdef PDF_STAT
            float pdf_stat;
            MapIf<float>& pdf;
        #endif
        #ifdef CIRC_DIV
            vector<float>& prob_thresholds;
            vector<int> dist_thresholds;
            char cur_region;
            MapIf<int> &divided_map;
        #endif
        #ifdef CWAVE_DEBUG
            CWaveDebugger &debugger;
        #endif
    };
    
    const bool IS_START=true;
    const bool IS_END=false;
    
    Galaxy calculate_distances(MapIf<int> &map, int x, int y
        #ifdef PDF_STAT
            , MapIf<float> & pdf
        #endif
        #ifdef CIRC_DIV
            , vector<float>& prob_thresholds
            , MapIf<int> &divided_map
        #endif
        #ifdef TRACK_MAP
            , MapIf<int> &track_map
        #endif
        #ifdef CWAVE_DEBUG
            , CWaveDebugger &debugger
        #endif
    );
    
    Galaxy calculate_distances(MapIf<int> &map, int x, int y, int max_dist
        #ifdef PDF_STAT
            , MapIf<float> & pdf
        #endif
        #ifdef CIRC_DIV
            , vector<float>& prob_thresholds
            , MapIf<int> &divided_map
        #endif
        #ifdef TRACK_MAP
            , MapIf<int> &track_map
        #endif
        #ifdef CWAVE_DEBUG
            , CWaveDebugger &debugger
        #endif
    );
    
    Galaxy galaxy_create(MapIf<int> &map
        #ifdef PDF_STAT
            , MapIf<float> & pdf
        #endif
        #ifdef CIRC_DIV
            , vector<float>& prob_thresholds
            , MapIf<int> &divided_map
        #endif        
        #ifdef TRACK_MAP
            , MapIf<int> &track_map
        #endif
        #ifdef CWAVE_DEBUG
            , CWaveDebugger &debugger
        #endif
    );
    
    Galaxy galaxy_create(MapIf<int> &map, int max_dist
        #ifdef PDF_STAT
            , MapIf<float> & pdf
        #endif
        #ifdef CIRC_DIV
            , vector<float>& prob_thresholds
            , MapIf<int> &divided_map
        #endif
        #ifdef TRACK_MAP
            , MapIf<int> &track_map
        #endif
        #ifdef CWAVE_DEBUG
            , CWaveDebugger &debugger
        #endif
    );
        
    
    void galaxy_add_star(Galaxy& glx, Star star);
    void galaxy_add_candidate(Galaxy& glx, const Candidate& c);
    void galaxy_big_bang(Galaxy& glx, int x, int y);    
    Star star_create(int x, int y, int init_dist);
    Star star_create_from_candidate(const Candidate& cand, int init_dist);
    void star_set_limited_range(Star& s, int sx, int sy, int ex, int ey);
    void star_grow(Galaxy& glx, Star& s);
    void star_delete_current_beam(Star &s);
    Boundary boundary_create(int x, int y, int r);
    bool boundary_add_pixel(Galaxy& glx, Star& star, Boundary& line);
#ifdef DEBUG
    void boundary_calculate_arc_tip(Galaxy& glx, Star& star, /**/ Boundary& b, int radius, ArcTip& output_tip);
    
    void galaxy_print(Galaxy& glx);
#else
    
    void boundary_calculate_arc_tip(Boundary& b, int radius, ArcTip& output_tip);
#endif    
    void beam_grow(Galaxy& glx, Star& s, /**/Beam& beam);
    void beam_arc_walk(Galaxy& glx, Star& star, Beam& beam, /**/ const Boundary& start, const Boundary& end);
    
    
    int  pixel_check_and_mark(Galaxy& glx, Star& star, char oct, int x, int y, bool is_nbp); // __attribute__((always_inline));
    bool pixel_pair_visit(Galaxy& glx, Star& star, Beam& beam, /**/int oct, int x, int y, int F, bool reg, bool nbp);
    bool pixel_in_beam_process(Galaxy& glx, Star& star, Beam& beam, /**/char oct, int x, int y, int F, int cost, bool with_nbp);
    void pixel_on_boundary_process(Galaxy& glx, Star& star, OctPoint pt, bool is_start);
    void pixel_on_boundary_process(Galaxy& glx, Star& star, OctPoint pt, bool is_start, bool inclsuive);

    
    #ifdef CWAVE_DEBUG
        enum PixelType {
            PIXEL_OVERLAP,
            PIXEL_WRONGDIST,
            PIXEL_EDGE,
            PIXEL_CANDIDATE,
            PIXEL_REGULAR,
            PIXEL_STARDEAD,
            PIXEL_STAR,
            PIXEL_BOUNDARY
        };
        
        class CWaveDebugger {
            public:
                virtual void highlight(Galaxy& glx, int x, int y, PixelType type)=0;
                virtual void star_print(Galaxy& glx, Star& s)=0;
                virtual void galaxy_print(Galaxy& glx)=0;
                virtual void map(Galaxy& glx)=0;
                virtual  void highlight_boundary(Galaxy& glx, Star& star, Boundary& line)=0;
        };
    #endif
}

#endif
