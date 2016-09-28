#include <ncurses.h>  // Documentation for NCurses: http://www.tldp.org/HOWTO/html_single/NCURSES-Programming-HOWTO
#include <stdlib.h> 
#include <math.h>  
#include <low_throughput_hmi/map.h>
#include <low_throughput_hmi/fast_dist.h>
#include <unistd.h>


#define DrawPixel(x, y, c) mvaddch(HEIGHT-(y), x, c); //refresh();//getch();
#define DrawCirclePixel(x, y) mvaddch(10-(y), x, '.'); 


using namespace low_throughput_hmi_cost;

void draw_circle(int x0, int y0, int radius);



#define H HEIGHT               
#define W WIDTH
#define CX int(WIDTH/2)
#define CY int(HEIGHT/2)
               
const int WIDTH=40, HEIGHT=WIDTH;

int main() {   
    
    initscr();          /* Start curses mode          */
    sleep(2);

    bool press_key = true;
        
    
    Map2D map(WIDTH, HEIGHT);
    map.display();
    Galaxy glx = galaxy_create(map);
    glx.ins = glx.stars.before_begin();
    int n_obstacles = 20;
    int radius = WIDTH/2-2;
    struct Point {int x; int y;};
    vector<vector<Point>> tests = {
        {{2,1}},
        {{4,3}},
        {{4,3}, {-1,9}},
        {{0,4}},        
        {{-1,4}},//nbp on new start boundary
        
        {{1,3}, {-2,-2}},//two points on the same arc
        {{6,4},{-7,2},{0,-7}},//three points on the same arc
        {{7,3},{-4,7},{0,-8}},
        {{7,3},{-6,-6},{0,-8}},//three points on the same arc, one is nbp
        {{5,0}},//in origin
        
        {{2,0}},
        {{1,0}},//in origin really close
        {{1,1},{-1,1}}, // 1px gate at r=1
        {{1,0},{0,1}},  // 1px gate at r=1.5
        {{-1,0},{0,-1}},
        
        {{3,3}},
        {{1,1}}, //45deg corner at r=1.5
        {{-1,1}},
        {{-7,2}}, //extra star points for detected for some reason
        {{2,1},{2,-1},{5,1},{5,-1},{8,1},{8,-1}}, // multiple gates
        
        {{-1,-1},{-1,2},{-4,-1},{3,8}, {-8,3},{-5,-7}}, // new test which failed (overlaps) 
        {{-1,-1},{-1,2}} //simpler version
    };

    int test=0;
    while (true) {
        if (test<tests.size()) {
            for (int w=0;w<tests[test].size();w++) {
                map.add_wall(CX+tests[test][w].x, CY+tests[test][w].y, 1);
            }
        } else {    
            map.add_obstacles(n_obstacles);
        }
        map.display();
        
        Star star = star_create(CX, CY, 0);
        for (int r=1; r<radius; r++) {
            star_grow(glx, star);
            map.display();
        }
        Boundary& start = star.beams.begin()->start;
        Boundary&   end = star.beams.begin()->end;
        if (press_key)    
            getch();
        map.clean();
        test++;
    }
    refresh();          /* Print it on to the real screen */
    endwin();           /* End curses mode        */

    return 0;

}

