#include <ncurses.h>  // Documentation for NCurses: http://www.tldp.org/HOWTO/html_single/NCURSES-Programming-HOWTO
#include <stdlib.h> 
#include <math.h>  
#include <vector>

const bool VISUAL=false;
const int MAX_RADIUS=300009; //1000;
const bool report_every_nbp         =false;
const bool report_left_diag_nbp     =false;
const bool report_right_diag_nbp    =true;
const bool report_vertical_nbps     =true;
const bool report_horizontal_nbps   =true;

const int TERM_HEIGHT=100;
const int COL_WIDTH=21;

using namespace std;

int WIDTH = MAX_RADIUS+1;
int HEIGHT = MAX_RADIUS+1;
vector<vector<bool>> map(WIDTH, vector<bool>(HEIGHT, false));

void draw_pixel(int x, int y, int r) {
    if (VISUAL) {
        mvaddch(y, x, 'X'); 
        refresh();
    }
    map[x][y]=true;
}


void draw_arc_in_first_two_octants(int x0, int y0, int r) {
    int x = r;
    int y = 0;
    int decisionOver2 = 1 - x;   // Decision criterion divided by 2 evaluated at x=r, y=0
    
    while( y<=x ) {
        draw_pixel(x0+x, y0+y, r);
        draw_pixel(x0+y, y0+x, r);
        y++;
        if (decisionOver2<0) { //used to be <=
            decisionOver2 += 2 * y + 1;   // Change in decision criterion for y -> y+1
        } else {
            x--;
            decisionOver2 += 2 * (y - x) + 1;   // Change for y -> y+1, x -> x-1
        }
    }
}

void key() {
    mvprintw(MAX_RADIUS+1, 0, " ");
    refresh();
    getch();
}

bool is_within_circle(int x, int y) {
    //calculate res = (x+1)*(x+1) + (y+1)*(x+1) - MAX_RADIUS*MAX_RADIUS
    long long tmp, res=0;    
    tmp = x+1; tmp *= tmp; res += tmp;
    tmp = y+1; tmp *= tmp; res += tmp;
    tmp = MAX_RADIUS; tmp *= tmp; res -= tmp;
    return (res<0);
}

int main() {
    initscr();          /* Start curses mode          */
    //getch();
    int d = VISUAL ? MAX_RADIUS+3 : 1;
    
    //draw arcs
    for (int r=1; r<=MAX_RADIUS; r++) {
        mvprintw(0, d, "Drawing arcs, r=%d", r); refresh();
        draw_arc_in_first_two_octants(0,0,r);
        //key();
    }
    
    //search for special NBPs
    int row=4, col=d;
    int c, c_ru, c_lu, c_r, c_u;
    mvprintw(1, d, "Check at x="); 
    for (int x=1;x<WIDTH-1;x++) {
        mvprintw(1, d+12, "%d", x); 
        refresh();
        
        for (int y=0;y<HEIGHT-1;y++) {
            if (x>y && is_within_circle(x,y) ) { 
                c = map[x][y];
                if (c==false) { // if this is NBP
                    c_ru = map[x+1][y+1];
                    c_lu = map[x-1][y+1];
                    c_r  = map[x+1][y];
                    c_u  = map[x][y+1];
                    
                    if (report_every_nbp) {
                        mvprintw(row++, col, "NBP:(%d,%d)=%d", x,y,c); refresh();
                        if (row==TERM_HEIGHT) {row=4;col+=COL_WIDTH;};
                        key();
                    }

                    /*if (look_for_inaccesible_diag_nbp && cl>0 && cl%2!=0 && x>y+1 && y-x/2>0) {
                        mvprintw(row++, col, "IL:(%d,%d)=%d,e=%d", x,y,c,y-x/2);
                        if (row==20) {row=1;col+=16;};
                        refresh();
                        mvprintw(0, 0, " ");getch();
                    }*/
                    if (report_left_diag_nbp && c_lu==false && y+1<x) {
                        mvprintw(row++, col, "LDIAG:(%d,%d)=%d", x,y,c); refresh();
                        if (row==TERM_HEIGHT) {row=4;col+=COL_WIDTH;};
                        key();
                    }
                    if (report_right_diag_nbp && c_ru==false && y+1<x) {
                        mvprintw(row++, col, "RDIAG:(%d,%d)=%d", x,y,c); refresh();
                        if (row==TERM_HEIGHT) {row=4;col+=COL_WIDTH;};
                        key();
                    } 

                    if (report_vertical_nbps && c_u==false ) {
                        mvprintw(row++, col, "VERT:(%d,%d)=%d", x,y,c); refresh();
                        if (row==TERM_HEIGHT) {row=4;col+=COL_WIDTH;};
                        key();
                    } 
                    
                    if (report_horizontal_nbps && c_r==false ) {
                        mvprintw(row++, col, "HOR:(%d,%d)=%d", x,y,c); refresh();
                        if (row==TERM_HEIGHT) {row=4;col+=COL_WIDTH;};
                        key();
                    }
                }
            }
        }
    }
    mvprintw(2, d, "DONE");

    refresh();          /* Print it on to the real screen */
    getch();            /* Wait for user input */
    endwin();           /* End curses mode        */

    return 0;
}