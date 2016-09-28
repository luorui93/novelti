#include <ncurses.h>  // Documentation for NCurses: http://www.tldp.org/HOWTO/html_single/NCURSES-Programming-HOWTO
#include <stdlib.h> 
#include <math.h>  
#include <low_throughput_hmi/map.h>
#include <low_throughput_hmi/fast_dist.h>



using namespace low_throughput_hmi_cost;

             
const int WIDTH=32003, HEIGHT=WIDTH;

int main() {   
    
    initscr();          /* Start curses mode          */

    bool press_key = true;
        
    
    Map2D map(WIDTH, HEIGHT);
    
    Galaxy glx = galaxy_create(map);
    int radius = WIDTH-1;
    Star star = star_create(0,0,0);
    star_set_limited_range(star, 10, 0, 10, 12);
    for (int r=0; r<radius; r++) {
        star_grow(glx, star);
        mvprintw(0, 0, "r=%d", r);
        refresh();
    }

    int row = 1, col=1, c, cr, cl;
    for (int x=1;x<WIDTH-1;x++) {
        mvprintw(0, 15, "x=%d", x); 
        refresh();
        for (int y=0;y<HEIGHT-1;y++) {
            c = map.m[x][y];
            cr = map.m[x+1][y+1];
            cl = map.m[x-1][y+1];
            if (c>0 && c%2!=0) {
                if (cl>0 && cl%2!=0 && x>y+1) {
                    mvprintw(row++, col, "CL:(%d,%d)=%d", x,y,c);
                    if (row==20) {row=1;col+=16;};
                    refresh();
                    mvprintw(0, 0, " ");getch();
                }
                if (cr>0 && cr%2!=0 && x>y+1) {
                    mvprintw(row++, col, "CR:(%d,%d)=%d", x,y,c);
                    if (row==20) {row=1;col+=16;};                    
                    refresh();
                    mvprintw(0, 0, " ");getch();
                }
            }
        }
    }
    mvprintw(0, 30, "DONE");
    refresh();
    getch();
    endwin();           /* End curses mode        */

    return 0;

}

