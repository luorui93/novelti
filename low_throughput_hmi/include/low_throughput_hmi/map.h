#ifndef MAP2D_H_
#define MAP2D_H_

#include <low_throughput_hmi/map_if.h>
#include <vector>
#include <ncurses.h>

#define MAP_CELL_EMPTY -1
#define MAP_CELL_OCCUPIED -10
#define MAP_CELL_OUT_OF_BOUNDARY -20
#define MAP_CELL_VISITED_OCCUPIED -1
#define MAP_CELL_VISITED_THRESHOLD -10
#define MAP_CELL_NOT_VISITED_OCCUPIED -20
#define MAP_CELL_NOT_VISITED -100
#define MAP_CELL_CENTER -30
//if cell<MAP_CELL_VISITED_THRESHOLD then it was never visited

using namespace std;

namespace low_throughput_hmi_cost {
   
    struct Wall {double x; double y; double l;};
    
    enum PixelColorPair { 
        PIXEL_COLOR_REGULAR   = 0,
        PIXEL_COLOR_EMPTY     = 0,
        PIXEL_COLOR_OCCUPIED  = 1,
        PIXEL_COLOR_BOUNDARY  = 2,
        PIXEL_COLOR_OVERLAP   = 3,
        PIXEL_COLOR_WRONGDIST = 4,
        PIXEL_COLOR_CANDIDATE = 5,
        PIXEL_COLOR_STAR      = 6,
        PIXEL_COLOR_STARDEAD  = 7,
        PIXEL_COLOR_EDGE      = 8,
        
        PIXEL_COLOR_BORDER    = 9,
        PIXEL_COLOR_REGION    = 20,
        
        
        PIXEL_COLOR_UNREACH    = 40,
        PIXEL_COLOR_REACH_NOT_VISITED    = 41,
        PIXEL_COLOR_REACH_VISITED    = 42
        
    }; 
    
    class Map2D : public MapIf<int> {
        private:
            int w;
            int h;        
        public:
            vector<vector<int>> m;
            vector<vector<int>> color;

 
            
            Map2D(int w, int h);
            ~Map2D();
            
            int get(int x, int y);              __attribute__((always_inline));
            void set(int x, int y, int val);    __attribute__((always_inline));
            int width();                            __attribute__((always_inline));
            int height();                            __attribute__((always_inline));
            
            void display();
            void display(int row, int col);
            void clean();
            void clean_dist();
            void add_obstacles(int number_of_pixels);
            void add_wall(int x0, int y0, int l);
            void add_walls(vector<Wall>& walls);
            void add_from_bits(bool** cell_traversable);
            void highlight(int x, int y, int color_pair_id);
            void resize(int w, int h, int val) {}; //TODO
            
    };
    
    
}
#endif
