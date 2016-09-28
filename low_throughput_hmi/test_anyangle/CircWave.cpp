/*
This code was developed by Dmitry Aleksndrovich Sinyukov (dmitry@sinyukov.com) at WPI.

Any distribution or usage of this code is only allowed with his explicit permission
Dmitry Aleksndrovich Sinyukov (dmitry@sinyukov.com)
*/

#include "CircWave.h"
#include <ncurses.h> 

//#define VISUAL 1
//#define OUTPUT_VALID_SCENE 1

#include "../src/fast_dist.cpp"
#include "AnyAngleAlgorithm.h"

#ifdef VISUAL
    #include "../src/map.cpp"
#else
    #include <low_throughput_hmi/map_1d_vector.cpp>
#endif


using namespace low_throughput_hmi_cost;

class CircWave : public AnyAngleAlgorithm {
    private:
        #ifdef VISUAL
            Map2D* map_;
        #else
            Map1D* map_;
        #endif
        int k;
        #ifdef OUTPUT_VALID_SCENE
            int n;
        #endif
        
            
    public:
        CircWave(std::vector<bool> &bits, int _width, int _height) :
            AnyAngleAlgorithm::AnyAngleAlgorithm(bits, _width, _height) {
            #ifdef VISUAL
                initscr();
                map_ = new Map2D(_width+2, _height+2);
                map_->add_from_bits(cell_traversable_);
                map_->display();
                //printf("444\n");
            #else
                map_ = new low_throughput_hmi_cost::Map1D(cell_traversable_, _width, _height);
            #endif
            k=0;
            #ifdef OUTPUT_VALID_SCENE
                printf("version 1\n");
                int n;
            #endif
        }
        
        ~CircWave() {
            delete(map_);
        }

        const std::string GetName() const {
            return "C-WAVE";
        }
        
        cost FindXYLocPath(xyLoc from, xyLoc to, std::vector<xyLoc> &path) {
            #ifdef OUTPUT_VALID_SCENE
                if (map_->get(from.x,from.y) != MAP_CELL_OCCUPIED) {
                    printf("%d\tmaps/dao/office4.map\t200\t140\t%d\t%d\t%d\t%d\t1.0000\n", n/10, from.x,from.y, to.x, to.y);
                    n++;
                } else {
                    //printf("====%d      %d\n",from.x,from.y);
                }
            #endif
            calculate_distances(*map_, from.x, from.y);
            k++;
            //printf("calculate_distances was called %d times\n", k);
            #ifdef VISUAL
                map_->display();
                //sleep(10);
            #else
            #endif
            map_->clean_dist();

            return 0;
        }


};

/*
>>> for k in range(0,120):
...     for n in range(0,10):
...             print "%d\tmaps/dao/office4.map\t200\t140\t%d\t%d\t%d\t%d\t1.0000" % (k, random.randint(3,196), random.randint(3,136), random.randint(3,196), random.randint(3,136))
... 

 */