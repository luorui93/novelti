#include <ncurses.h>  // Documentation for NCurses: http://www.tldp.org/HOWTO/html_single/NCURSES-Programming-HOWTO
#include <stdlib.h> 
#include <math.h>  
#include <string>
#include <set>
#include <low_throughput_hmi/map.h>

#define TRACK_MAP 1
#include <low_throughput_hmi/fast_dist.h>
#include <low_throughput_hmi/space_divider.h>
#include "test_mazes.cpp"


#include <time.h>       /* time */

using namespace low_throughput_hmi_cost;
               
const int W=30, H=W, CX=int(W/2), CY=int(H/2);

struct Point {int x; int y;};

map<const char*, vector<Wall> > mazes = generate_test_mazes(W, H);
 
void keyboard_move(Map2D& map, Point& c) {
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


void test_0(Map2D& map, const char*  maze_index, int distance_threshold) {
    Point c={10,10};//{1,8};//{3,11};
    
    while(true) {
            
        mvprintw(map.height()+1, 0, "Current center point: (%d,%d)", c.x, c.y);
        map.add_walls(mazes[maze_index]);
        if (map.m[c.x][c.y]==MAP_CELL_EMPTY) {
            Map2D track_map(map.width(), map.height());
            Galaxy glx = calculate_distances(map, c.x, c.y, track_map);
            map.display();
            track_map.display(glx.track_map.height()+5, 0);
            map_divide({0.25, 0.25, 0.25, 0.25}, map, track_map, glx.track_stars, c.x, c.y);
            map.clean();
        }
        
        keyboard_move(map, c);
    }
}




int main() {   

    sleep(2);
    initscr();
    
    Map2D map(W, H);
    init_pair(10, COLOR_RED,     COLOR_BLUE);

    
    int test_type=0;
    
    switch(test_type) {
        case 0: test_0(map, "pi", 8); break;//maze_index, distance_threshold
    }
    getch();
    refresh();
    endwin();

    return 0;

}
