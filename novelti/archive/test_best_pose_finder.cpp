#include <ncurses.h>  // Documentation for NCurses: http://www.tldp.org/HOWTO/html_single/NCURSES-Programming-HOWTO
#include <stdlib.h> 
#include <math.h>  
#include <string>
#include <set>
#include <novelti/map.h>

#include <novelti/fast_dist.h>
#include <novelti/best_pose_finder.h>
#include "test_mazes.cpp"


#include <time.h>       /* time */

using namespace novelti;
               
const int W=30, H=W, CX=int(W/2), CY=int(H/2);

struct Point {int x; int y;};

map<const char*, vector<Wall> > mazes = generate_test_mazes(W, H);
 
void keyboard_move(Map2D& map, Point2D& c) {
    if (getch() == '\033') { // if the first value is esc
        getch(); // skip the [
        switch(getch()) { // the real value
            case 'A':
                if ( c.y+1<map.height() )  c.y++;
                break;
            case 'B':
                if ( c.y-1>=0 )  c.y--;
                break;
            case 'C':
                if ( c.x+1<map.width() )  c.x++;
                break;
            case 'D':
                if ( c.x-1>=0 )  c.x--;
                break;
        }
    }
}


void test_0(Point2D map_size, const char*  map_name, int max_dist, Point2D cur_pose) {
    //{8,10};///{8,10};//{1,8};//{3,11};
    
    Map2D map(map_size.x, map_size.y);
    map.add_walls(mazes[map_name]);
    
    MapRos<novelti::FloatMap,float>  pdf(map.width(), map.height(), (1.0/map.width())/map.height());
    
    while(true) {
            
        mvprintw(map.height()+1, 0, "Current robot pose: (%d,%d)", cur_pose.x, cur_pose.y);
        map.display();
        if (map.m[cur_pose.x][cur_pose.y]==MAP_CELL_EMPTY) {
            Point2D p = find_best_pose(map, pdf, cur_pose, max_dist);
             mvprintw(map.height()+7, 0, "best point is (%d,%d) on the map", p.x, p.y);
        }
        
        keyboard_move(map, cur_pose);
    }
}




int main() {   

    sleep(2);
    initscr();
    
    init_pair(10, COLOR_RED,     COLOR_BLUE);

    
    int test_type=0;
    
    switch(test_type) {
                     //map_size     map_name        max_dist    start_point
        case 0: test_0({5, 9},      "multi_peak",   38,         {1,1}); break;//minimal example with two minimums            
        case 1: test_0({30, 30},    "pi",           8,          {1,1}); break;//maze_index, distance_threshold
    }
    getch();
    refresh();
    endwin();

    return 0;

}
