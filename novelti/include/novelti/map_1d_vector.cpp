#ifndef MAP_1D_H_
#define MAP_1D_H_


#include <novelti/map_if.h>

using namespace std;

namespace novelti {
    
    #define MAP_CELL_EMPTY -1
    #define MAP_CELL_OCCUPIED -10
    
    class Map1D : public MapIf<int> {
        private:
            int* data_;
            int width_;
            int height_;
            
        public:
            Map1D(bool** cell_traversable, int width, int height) {
                width_ = width;
                height_ = height;
                data_ = new int[width_*height_];
                for (int y=0; y<height_; y++)
                    for (int x=0; x<width_; x++)
                        if (cell_traversable[x][y])
                            set(x,y,MAP_CELL_EMPTY);
                        else
                            set(x,y,MAP_CELL_OCCUPIED);
            }
            
            ~Map1D() {
                delete(data_);
            };
            void resize(int w, int h, int val) {}; //TODO
            int get(int x, int y) { return data_[x+y*width_];};           __attribute__((always_inline));
            void set(int x, int y, int val) {data_[x+y*width_] = val;};   __attribute__((always_inline));
            int width()     { return width_; };         __attribute__((always_inline));
            int height()    { return height_; };        __attribute__((always_inline));
            void clean_dist() { 
                for(int k=0; k<width_*height_; k++)
                    if (data_[k]>=0)
                        data_[k] = MAP_CELL_EMPTY;
            };
    };
    
    
}
#endif
