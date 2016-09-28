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
               
const int WIDTH=40, HEIGHT=40;

int main() {   
    
    initscr();          /* Start curses mode          */
    sleep(2);

    bool press_key = true;
        
    
    Map2D map(WIDTH, HEIGHT);
    map.display();
    Galaxy glx = galaxy_create(map);
    glx.ins = glx.stars.before_begin();
    double a1,a2;
    int n=100;
    double radius = 14;
    for (int k1=0; k1<n; k1++) { //19,33
        for (int k2=k1; k2<n; k2++) {
            a1=k1*2*M_PI/n; 
            a2=k2*2*M_PI/n;
            Star star = star_create(CX, CY, 0);
            //star_print(glx, star);
            star_set_limited_range(star, int(radius*cos(a1)), int(radius*sin(a1)), int(radius*cos(a2)), int(radius*sin(a2)));
            for (int r=1; r<radius; r++) {
                star_grow(glx, star);
                map.display();
            }
            Boundary& start = star.beams.begin()->start;
            Boundary&   end = star.beams.begin()->end;
            mvprintw(0, 0, "(k1,k2)=(%d,%d)\t(a1,a2)=(%f,%f)", k1, k2, a1, a2);
            mvprintw(1,0, "     start(octant,dx,dy)=(%d,%d,%d)", start.octant, start.dx, start.dy);
            mvprintw(2,0, "       end(octant,dx,dy)=(%d,%d,%d)", end.octant, end.dx, end.dy);
            if (press_key)    
                getch();
            map.clean();
        }
    }
    refresh();          /* Print it on to the real screen */
    endwin();           /* End curses mode        */

    return 0;

}

