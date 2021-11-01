#ifndef COMPOUNDMAP_H_
#define COMPOUNDMAP_H_

#ifdef MULTITHREAD
    #include <atomic>
#endif
#include <limits>
#include <vector>
#include <string>


#ifdef CWAVE2_FLOAT_MERGE
    #define CWAVE2_FLOAT_MERGE_OFFSET 2
    #include <math.h>
#else
    #define CWAVE2_FLOAT_MERGE_OFFSET 1
#endif

#define SEPARATE 1
using namespace std;

/*
                          CELL x-coordinate
              0       1       2       3       4       5
        4 +-------+-------+-------+-------+-------+-------+
          |       |       |       |       |       |       |
 V  y     |       |       |       |       |       |       |  3
 E  -     |       |       |       |       |       |       |
 R  c   3 +-------+-------+-------+-------+-------+-------+     C  y
 T  o     |       |       |       |XXXXXXX|XXXXXXX|       |     E  -
 E  o     |       |       |       |XXXXXXX|XXXXXXX|       |  2  L  c
 X  r     |       |       |       |XXXXXXX|XXXXXXX|       |     L  o
    d   2 +-------+-------+-------+-------+-------+-------+        o
    i     |       |       |       |XXXXXXX|XXXXXXX|       |        r
    n     |       |       |       |XXXXXXX|XXXXXXX|       |  1     d
    a     |       |       |       |XXXXXXX|XXXXXXX|       |        i
    t   1 +-------+-------+-------+-------+-------+-------+        n
    e     |       |       |       |       |       |       |        a
          |       |       |       |       |       |       |  0     t
          |       |       |       |       |       |       |
        0 +-------+-------+-------+-------+-------+-------+
          0       1       2       3       4       5       6 
                        VERTEX x-coordinate

Compound map resolution is defined for vertices:
    width_ = 7
    height_ =  5

Cell can be free ([0,0], [2,1]...) or occupied ([3,1], [4,1], [3,2], [4,2])
If vertex is surrounded by 4-occupied cells (e.g., [4,2]), it is called "blocked", otherwise it is called unblocked (all other vertices on this map)
*/


namespace cwave {

    #ifdef CWAVE2_TRACK
        typedef struct {
            int x;
            int y;
            double dist;
        } TrackStar;
    #endif
    
    const bool OCCUPIED = true;
    const bool FREED    = false;
    
    typedef int cell;
    #ifdef MULTITHREAD
        typedef atomic<int> array_cell;
    #else
        typedef int array_cell;
    #endif
    
    const int MAP_POINT_UNEXPLORED = std::numeric_limits<int>::max();
    
    class CompoundMap {
        public:
            static const int mask1000 = ~std::numeric_limits<int>::max();
            static const int mask0111 =  std::numeric_limits<int>::max();
            
            int width_;
            int height_;
            array_cell *data_;
            #ifdef SEPARATE
                bool *cells_;
            #endif
            #ifdef  CWAVE2_TRACK
                vector<TrackStar> track_stars;
                vector<int> track_map;
                void addTrackStar(int x, int y, double dist);
                void updTrackMap(int x, int y, int star_id);
                int getTrackStarId(int x, int y);
                TrackStar getTrackStar(int star_id);
                TrackStar getTrackStar(int x, int y);
                #ifdef CWAVE2_FLOAT_MERGE
                    double getExactDist(int x, int y);
                #endif
                //void clearTrack();
            #endif
        
            CompoundMap(){};
            void construct(int width, int height);
            void construct(FILE *stream);
            CompoundMap(FILE *stream);
            CompoundMap(int width, int height);
            CompoundMap(string fname);
            CompoundMap(bool** cell_traversable, int width, int height);
            CompoundMap(const CompoundMap &c);
            //void set(int x, int y, int val);
            //cell  get(int x, int y);
            
            void setPixel(int x, int y, bool is_occupied);
            bool isPixelOccupied(int x, int y);
            int  getPoint(int x, int y);
            void setPoint(int x, int y, int dist);
            int  updatePoint(int x, int y, int dist);
            int  width();
            int  height();
            void clearDist();
            bool isSurroundedByObstacles(int x, int y);
            bool areAllPointsExplored(int *fail_x, int *fail_y);
            void save(string& fname);
            ~CompoundMap();
    };
    
}
#endif
