#ifndef MAP_ROS_H_
#define MAP_ROS_H_

#include <novelti/map.h>
#include <vector>


using namespace std;

namespace novelti {
   
    template <class RosMsg1, class CellType> //MapRos<nav_msgs::OccupancyGrid,int>    MapRos<FloatMap,float>    
    class MapRos : public MapIf<CellType> {
        private:
            RosMsg1* msg;
        public:
            MapRos() {
                msg = NULL;
            }
            MapRos(RosMsg1& grid_msg) {
                msg = new RosMsg1(grid_msg);
            }
            
            MapRos(int w, int h, CellType init_val) {
                msg = new RosMsg1();
                msg->info.width = w;
                msg->info.height = h;
                msg->data = vector<CellType>(w*h, init_val);                
            }
            

            
            CellType  get(int x, int y)           { return msg->data[x+y*msg->info.width]; };              __attribute__((always_inline));
            void set(int x, int y, CellType val)  { msg->data[x+y*msg->info.width]=val; };    __attribute__((always_inline));
            int  width()                     { return msg->info.width; };                            __attribute__((always_inline));
            int  height()                    { return msg->info.height; };    
            
            MapRos(MapIf<CellType>& m) {
                msg = new RosMsg1();
                msg->info.width = m.width();
                msg->info.height = m.height();
                msg->data = vector<CellType>(m.width()*m.height(), 0);
                for (int x=0; x<m.width(); x++)
                    for (int y=0; y<m.height(); y++)
                        set(x,y, m.get(x,y));
            }
            
            float  resolution()                    { return msg->info.resolution; };       __attribute__((always_inline));
            
            void resize(int w, int h, CellType val) {
                msg->info.width = w;
                msg->info.height = h;
                msg->data = vector<CellType>(w*h, val);
            }
            
            void clean_dist() { 
                for(int k=0; k<msg->info.width*msg->info.height; k++)
                    if (msg->data[k]>=0)
                        msg->data[k] = MAP_CELL_EMPTY;
            };
            RosMsg1* get_msg() {
                return msg;
            }
            ~MapRos() {
                delete(msg);
            };
            
            void add_wall(int x0, int y0, int l) {
                if (l>0)
                    for (int x=x0; x<x0+l; x++) {
                        set(x,y0, MAP_CELL_OCCUPIED);
                    }
                else
                    for (int y=y0; y<y0-l; y++) {
                        set(x0,y, MAP_CELL_OCCUPIED);
                    }
            }
            
            void add_walls(vector<Wall>& walls) {
                for (int w=0;w<walls.size();w++) {
                    add_wall(int(walls[w].x), int(walls[w].y), int(walls[w].l));
                }
            }
    };
}
#endif
