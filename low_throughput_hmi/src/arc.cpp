#include <ncurses.h>  // Documentation for NCurses: http://www.tldp.org/HOWTO/html_single/NCURSES-Programming-HOWTO
#include <stdlib.h> 
#include <math.h>  
#include <low_throughput_hmi/map.h>
#include <low_throughput_hmi/arc_list.h>


#define DrawPixel(x, y, c) mvaddch(HEIGHT-(y), x, c); //refresh();//getch();
#define DrawCirclePixel(x, y) mvaddch(10-(y), x, '.'); 

#define octant_from_xy(x,y,abs_x,abs_y) \
        (abs_x)>=(abs_y)\
            ? (x)!=(y)\
                ? (x)>0\
                    ? ((y)>=0 ? 0 : 7)\
                    : ((y)>0  ? 3 : 4)\
                : ((x)>0 ? 1 : 5)\
            : (y)>=0\
                ? ((x)>0  ? 1 : 2)\
                : ((x)>=0 ? 6 : 5);\

/*#define next_line_pix

octant = octant_from_xy(x,y,abs_x,abs_y)
switch(octant) {
    case 0: next_line_pix(x,y); break;
    case 1: next_line_pix(y,x); break;
    case 2: next_line_pix(y,-x); break;
    case 3: next_line_pix(-x,y); break;
    case 4: next_line_pix(-x,-y); break;
    case 5: next_line_pix(-y,-x); break;
    case 6: next_line_pix(-y,x); break;
    case 7: next_line_pix(x,-y); break;
}*/

using namespace low_throughput_hmi_cost;

void draw_circle(int x0, int y0, int radius);


double constrain_angle(double);

#define H HEIGHT               
#define W WIDTH
               
const int WIDTH=80, HEIGHT=24;

/*
void qqq(orientation& line) {
    
}*/

    forward_list<ArcList>::iterator iter ,iterA, iterB, iterC, iter4insert, prev1, prev2, next_elm;//
     // iter4insert points to the element which is right before the first element
     // whose init_dist>=focuses.begin().cur_cost+2
     // iter C points to the first element whose init_dist > focuses.begin().cur_cost
    forward_list<ArcList> focuses;
    int next_dist, cur_dist;
    void printFocuses() {
        int k = 2, col=W+35;
        
        mvprintw(k++, col, "========================");
        for (forward_list<ArcList>::iterator it=focuses.begin();it!=focuses.end();++it) {
            char C_stat = ' ', D_stat = ' ';
            if (it==iterC) C_stat='C';
            if (it==iter4insert) D_stat='D';
            mvprintw(k++, col, " %c%c%c(%d, %d)%c=> %d+%d=%d    ", 
                     C_stat, D_stat, 
                     it==iter? '>': ' ', 
                     it->cx_, it->cy_, 
                     it->limits_.empty()? 'E': ' ', 
                     it->init_dist_, it->radius_, it->cur_dist_);
            refresh();
        }
        mvprintw(k++, col, "==========%d==========");
        mvprintw(k++, col, "cur_dist =%d           ", cur_dist);
        mvprintw(k++, col, "next_dist=%d           ", next_dist);
        refresh();
    }

int main() {   
    
    initscr();          /* Start curses mode          */
    //getch();

    sleep(2);
    //Generate map
    Map2D map(WIDTH, HEIGHT);
    //vector<vector<int>> map(WIDTH, vector<int>(HEIGHT, MAP_CELL_EMPTY));
    //map.add_obstacles(30);
    vector<vector<int>> walls{
        {0,0,W}, {0,0,-H}, {W-1,0,-H},{0,H-1,W}, //main rectangle
        {W/4-4, 3*H/4, W/4}, {W/4-4, 1*H/4, W/4}, {W/2-4, H/4, -H/2-1}, // =|
        {W/4-7, 3*H/4-3, W/4}, {W/4-7, 1*H/4+3, W/4}, {W/4-7, H/4+3, -(H/2+1-6)}//,
        //{W/2+19, H/2, 1}//, {W/2, H/2+1, 1}, {W/2-1, H/2, 1}, {W/2, H/2-1, 1}
    };
    for (auto w: walls)
        map.add_wall(w[0],w[1],w[2]);
    map.display();
    

    ArcList first_focus(map, focuses, int(WIDTH/2), int(HEIGHT/2), 0, 0.0, 7.0);
    //list.addArc(-2.1, -0.5);limits_
    

    bool iterC_defined=false, iter4insert_defined=false;
    iter = focuses.before_begin();
    while (!first_focus.limits_.empty()) {
        mvaddch(0,0,'0'); getch(); 
        first_focus.grow(iter);
        iter = first_focus.where_to_insert_;
        next_elm = iter; ++next_elm;
        if (!focuses.empty()) {
            if (!iterC_defined && iter->init_dist_ > focuses.begin()->init_dist_) {
                iterC=iter;
                iterC_defined =true;
                next_dist = iterC->cur_dist_;
            }
            if (!iter4insert_defined && ( next_elm==focuses.cend() || next_elm->init_dist_ - focuses.begin()->init_dist_>=2)) {
                iter4insert=iter;
                iter4insert_defined = true;
            }
        }
        map.display();
    }
    
    //printFocuses();

    
    while (!focuses.empty()) {
        //mvaddch(0,0,'0'); getch();
        cur_dist = focuses.begin()->cur_dist_;
        printFocuses();
        //for (;cur_dist<next_dist; cur_dist++) {
            //iter = focuses.begin();prev1=focuses.before_begin();
            //do {
            for (iter = focuses.begin(), prev1=focuses.before_begin(); iter!=focuses.end() && iter->cur_dist_<=cur_dist;  ) {
                mvaddch(0,0,'0'); getch();
                printFocuses();
                iter->grow(iter4insert);
                printFocuses();
                iter4insert=iter->where_to_insert_;
                if (iter->limits_.empty()) {
                    if (iter4insert==iter) iter4insert=prev1;
                    ++iter;
                    if (iter==focuses.end() && prev1!=focuses.before_begin()) break;
                    focuses.erase_after(prev1);
                    if (focuses.empty()) goto finished;
                    //iter=prev1;
                }
                printFocuses();
               if (iter==focuses.end()) {
                   iter4insert_defined=false;
               }
               prev1=iter;
               ++iter;    
            }// while (iter!=iterB)
            //update iter4insert (iterD)
            printFocuses();
            if (iter4insert!= focuses.end() && iter4insert->cur_dist_ < cur_dist+2) {
                do {
                    prev2=iter4insert;
                    ++iter4insert;
                } while(iter4insert!= focuses.end() && iter4insert->cur_dist_ < cur_dist+2);
                iter4insert=prev2;        
            };
            printFocuses();
        //}
        //while (iterC!=focuses.end() && iterC->cur_dist_<=next_dist) ++iterC;
        //next_dist = iterC->cur_dist_;
    }
    finished:
    mvaddch(0,0,'!'); 
    /*
    for (int r=1; r<40; r++) {
        mvaddch(0,0,'0'); getch(); 
        list.drawAt(r);
        map.display();
        //getch();
    }*/
        
    
/*    
    double sa, ea;
    int k=0;
    for (double deg=0; deg<360; deg+=45) {
        mvprintw(1, 1, "deg=%f", deg);
        sa = constrain_angle(deg*3.14159/180);
        ea = constrain_angle((deg+350)*3.14159/180);
        mvprintw(2, 1, "sa=%f, ea=%f", sa, ea);
        draw_circle(int ((deg-32)/2)+36, 0, 7);
        mvprintw(3, 1, "sa=%f, ea=%f", sa, ea);
        draw_arc(int ((deg-32)/2)+36, 0, 7, sa,ea);
        mvaddch(0, 0, 'c');
        getch();
    }
  */  
    /*for (int r=1;r<200;r++){
        draw_circle(80, -40, r);
        mvaddch(0, 0, 'c');
        getch();
    } */
        
    //mvprintw(20, 5, "round(sx)=%d, round(sy)=%d", int(round(sx)), int(round(sy)));

    refresh();          /* Print it on to the real screen */
    getch();            /* Wait for user input */
    endwin();           /* End curses mode        */

    return 0;

}

double constrain_angle(double x){
    x = fmod(x,2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    return x;
}



void draw_circle(int x0, int y0, int radius) {
    int x = radius;
    int y = 0;
    int decisionOver2 = 1 - x;   // Decision criterion divided by 2 evaluated at x=r, y=0
    
    while( y<=x ) {
        DrawCirclePixel( x + x0,  y + y0);
        DrawCirclePixel( y + x0,  x + y0);
        DrawCirclePixel(-x + x0,  y + y0);
        DrawCirclePixel(-y + x0,  x + y0);
        
        DrawCirclePixel(-x + x0, -y + y0);
        DrawCirclePixel(-y + x0, -x + y0);
        DrawCirclePixel( x + x0, -y + y0);
        DrawCirclePixel( y + x0, -x + y0);
        y++;
        if (decisionOver2<=0) {
            decisionOver2 += 2 * y + 1;   // Change in decision criterion for y -> y+1
        } else {
            x--;
            decisionOver2 += 2 * (y - x) + 1;   // Change for y -> y+1, x -> x-1
        }
    }
}