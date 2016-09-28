#include <ncurses.h>  // Documentation for NCurses: http://www.tldp.org/HOWTO/html_single/NCURSES-Programming-HOWTO
#include <stdlib.h> 
#include <math.h>  
#include <string>
#include <set>
#include <low_throughput_hmi/map.h>
#ifdef CWAVE_DEBUG
    #include "cwave_text_debugger.cpp"
#endif
#include <low_throughput_hmi/fast_dist.h>
#include "test_mazes.cpp"


#include <time.h>       /* time */

using namespace low_throughput_hmi_cost;
               
const int W=70, H=W, CX=int(W/2), CY=int(H/2);

struct Point {int x; int y;};

inline bool operator<(const Point& lhs, const Point& rhs) {
        return ((lhs.x<<sizeof(int)*8+lhs.y)<(rhs.x<<sizeof(int)*8+rhs.y));
    }

  
map<const char*, vector<Wall> > mazes = generate_test_mazes(W, H);
 
Map2D* my_map;
    
void test_manual_test_switch(Galaxy& glx, bool wait_for_key, int test0) {
    int test=test0;
    
    struct Test {
        std::string name; 
        double x; 
        double y; 
        vector<Wall> maze;
        Point test_point;
    };

    vector<Test> tests = {
        
{"Test N1: Abrubt change: center=(22,5)->(22,6), 153->144, pixel=(51,67). Move=UP", 22,5,  mazes["rooms_with_1px_doors"], {51,67}}, {"Test N2: Abrubt change: center=(22,5)->(22,6), 153->144, pixel=(51,67). Move=UP", 22,6,  mazes["rooms_with_1px_doors"], {51,67}},
{"Test N1: Abrubt change: center=(22,6)->(22,5), 144->153, pixel=(51,67). Move=DOWN", 22,6,  mazes["rooms_with_1px_doors"], {51,67}}, {"Test N2: Abrubt change: center=(22,6)->(22,5), 144->153, pixel=(51,67). Move=DOWN", 22,5,  mazes["rooms_with_1px_doors"], {51,67}},

        
        {"Just 4 walls", CX, CY, mazes["4walls"]},
        {"Test 1", CX, CY, mazes["1pt"]},
        {"Test 2", 0.1*W, 0.9*H, mazes["pi"]},
        {"Test 3", 0.3*W, 0.4*H, mazes["pi"]},
        {"Test 4", 0.6*W, 0.1*H, mazes["pi"]},
        {"Test 5", 0.8*W, 0.7*H, mazes["pi"]},
        
        //{"Test 6", 0.5*W, 0.5*H,  mazes["4rooms"]},
        {"Test 6", 0.6*W, 0.5*H,  mazes["4rooms"]},
        {"Test 7", 0.3*W, 0.4*H,  mazes["4rooms"]},
        {"Test 8", 0.6*W, 0.1*H,  mazes["4rooms"]},
        {"Test 9", 0.14*W, 0.7*H,  mazes["4rooms"]},
        {"Test 10", 32, 27,  mazes["4rooms"]},
        {"Test 11", 24, 35,  mazes["4rooms"]},
        {"Test 12", 28, 25,  mazes["4rooms"]},
        {"Test 13", 34, 25,  mazes["4rooms"]},
        {"Test 14", 54, 35,  mazes["4rooms"]},
        {"Test 15", 36, 32,  mazes["4rooms"]},
        {"Test 16", 24, 36,  mazes["4rooms"]},
        {"Test 17", 16, 11,  mazes["4rooms"]},
        {"Test 18", 24-6, 36+30,  mazes["4rooms2"]},
        
        {"Test 19", 27,27,  mazes["4rooms2"]},
        {"Test 20", 16,36,  mazes["4rooms2"]},
        {"Test 21", 31,22,  mazes["4rooms2"]},
        {"Test 22", 24,36,  mazes["4rooms2"]},
        {"Test 23", 24,30,  mazes["4rooms2"]},
        {"Test 24", 29,25,  mazes["4rooms2"]},
        {"Test 25", 17,37,  mazes["4rooms2"]},
        {"Test 26", 19,35,  mazes["4rooms2"]},
        {"Test 27", 19,34,  mazes["4rooms2"]},

        {"Test N1: (53,54)=>(52,54) @ (50,6)", 53,54,  mazes["4rooms2"]}, {"Test N2: (53,54)=>(52,54) @ (50,6)", 52,54,  mazes["4rooms2"]}, 
                
        {"Test N1: (12,66):166 (11,66):173 @(51,31)", 12,66,  mazes["4rooms2"]}, {"Test N2: (12,66):166 (11,66):173 @(51,31)", 11,66,  mazes["4rooms2"]}, 
        {"Test N1: (13,65):150 (12,65):157 @(42,30)", 13,65,  mazes["4rooms2"]}, {"Test N2: (13,65):150 (12,65):157 @(42,30)", 12,65,  mazes["4rooms2"]}, 
        {"Test N1: (10,65):160 (9,65):167 @(46,33)", 10,65,  mazes["4rooms2"]}, {"Test N2: (10,65):160 (9,65):167 @(46,33)", 9,65,  mazes["4rooms2"]}, 
        {"Test N1: (7,65):158 (6,65):165 @(42,34)", 7,65,  mazes["4rooms2"]}, {"Test N2: (7,65):158 (6,65):165 @(42,34)", 6,65,  mazes["4rooms2"]}, 
        {"Test N1: (4,65):160 (3,65):167 @(41,36)", 4,65,  mazes["4rooms2"]}, {"Test N2: (4,65):160 (3,65):167 @(41,36)", 3,65,  mazes["4rooms2"]}, 
        {"Test N1: (13,61):152 (12,61):159 @(46,33)", 13,61,  mazes["4rooms2"]}, {"Test N2: (13,61):152 (12,61):159 @(46,33)", 12,61,  mazes["4rooms2"]}, 
        {"Test N1: (9,66):166 (8,66):173 @(49,34)", 9,66,  mazes["4rooms2"]}, {"Test N2: (9,66):166 (8,66):173 @(49,34)", 8,66,  mazes["4rooms2"]}, 
        {"Test N1: (2,67):164 (1,67):171 @(41,36)", 2,67,  mazes["4rooms2"]}, {"Test N2: (2,67):164 (1,67):171 @(41,36)", 1,67,  mazes["4rooms2"]},         
                
                
        {"Test N1: (37,35):136 (38,35):142 @(13,56)", 37,35,  mazes["4rooms2"]}, {"Test N2: (37,35):136 (38,35):142 @(13,56)", 38,35,  mazes["4rooms2"]}, 
        {"Test N1: (38,31):20 (38,30):25 @(33,39)", 38,31,  mazes["4rooms2"]}, {"Test N2: (38,31):20 (38,30):25 @(33,39)", 38,30,  mazes["4rooms2"]}, 
        {"Test N1: (31,20):6 (31,19):11 @(31,22)", 31,20,  mazes["4rooms2"]}, {"Test N2: (31,20):6 (31,19):11 @(31,22)", 31,19,  mazes["4rooms2"]}, 
                
        {"Test N1: (53,54)=>(52,54) @ (50,6)", 53,54,  mazes["4rooms2"]}, {"Test N2: (53,54)=>(52,54) @ (50,6)", 52,54,  mazes["4rooms2"]}, 
        
        {"Test NEW STAR placement", 10,30,  mazes["rooms_with_1px_doors"]},
        {"Test NEW STAR placement 2", 1,3,  mazes["rooms_with_1px_doors"]}, 
        {"Test NEW STAR placement 3", 1,20,  mazes["rooms_with_1px_doors"]}, 
    };
    
    while (test<tests.size() && test>=0) {
        my_map->add_walls(tests[test].maze);
        mvprintw(my_map->height()+1, 0, "Test = %d   %s                  ", test, tests[test].name.c_str());
        galaxy_big_bang(glx, int(tests[test].x), int(tests[test].y));
        my_map->highlight(tests[test].test_point.x, tests[test].test_point.y, 10);
        my_map->display();        
        if (wait_for_key)    
            if (getch() == '\033') { // if the first value is esc
                getch(); // skip the [
                switch(getch()) { // the real value
                    case 'C':
                        test++;
                        break;
                    case 'D':
                        test--;
                        break;
                }
            }
        my_map->clean();
    }
};



void test_random_motion(Galaxy& glx, const char* maze_index, bool wait_for_key) {
    
    /* Regular expressions to make static tests:
     * Find: .*\|\((\d+),(\d+)\)=>\((\d+),(\d+)\).*
     * Replace: {"Test N1: \1", \2,\3,  mazes["4rooms2"]}, {"Test N2: \1", \4,\5,  mazes["4rooms2"]}, 
     */
    std::set<Point> unfilled_centers;
    std::set<Point> abrupt_change;
    std::vector<std::vector<int>> prev;
    bool prev_defined=false;
    int x=CX, y=CY, x1=x,y1=y, dir=0, row=2, row2=2, row3=2, row4=2;
    int prev_x1, prev_y1;
    mvprintw(0, my_map->width()+1, "==========Unfilled areas========== ================Abrubt change in distance=================");
    mvprintw(1, my_map->width()+1, "       all      |      unique                  all          |        unique      ");
    srand(time(NULL));
    while (true) {
        my_map->add_walls(mazes[maze_index]);

        if (rand()%100<10) {
            dir=rand()%4;
        }
        switch (dir) {
                case 0: x1=x+1; break;
                case 1: x1=x-1; break;
                case 2: y1=y+1; break;
                case 3: y1=y-1; break;
        };
        if (my_map->m[x1][y1]==MAP_CELL_EMPTY) {
            mvprintw(my_map->height()+1, 0, "Current center point: (%d,%d)", x1, y1);
            refresh();
            x=x1; y=y1;
            galaxy_big_bang(glx, x1, y1);
            my_map->display();
            for (int tx=0; tx<my_map->width()-1;tx++)
                for (int ty=0; ty<my_map->height();ty++) {
                    if (my_map->m[tx][ty]==MAP_CELL_EMPTY && my_map->m[tx+1][ty]==MAP_CELL_EMPTY) {
                        mvprintw(row++, my_map->width()+1, "(%d,%d)=>(%d,%d)", x1, y1, tx,ty);
                        auto ret = unfilled_centers.insert({x1,y1});
                        if (ret.second) {
                            mvprintw(row2++, my_map->width()+1+17, "(%d,%d)=>(%d,%d)", x1, y1, tx,ty);
                            refresh();    
                        }
                    }
                    if (prev_defined && my_map->m[tx][ty]>=0 && prev[tx][ty]>=0 && my_map->m[tx][ty]-prev[tx][ty] > 6) {
                        mvprintw(row3++, my_map->width()+1+35, "(%d,%d)>(%d,%d) @(%d,%d)", prev_x1,prev_y1, x1,y1, tx,ty);
                        auto ret = abrupt_change.insert({x1,y1});
                        if (ret.second) {
                            mvprintw(row4++, my_map->width()+1+35+25, "|(%d,%d):%d (%d,%d):%d @(%d,%d)", 
                                     prev_x1,prev_y1, prev[tx][ty],
                                     x1,y1, my_map->m[tx][ty],
                                     tx,ty);
                            refresh();    
                        }                        
                    }
                }
            prev_x1 = x1; prev_y1 = y1;
            prev = my_map->m;
            prev_defined=true;
            if (wait_for_key)    
                getch();
            my_map->clean();     
        }
    }
}
    




void test_all_pixels(Galaxy& glx, const char* maze_index, int distance_threshold, bool display_all) {
    /* Regexps to convert to static tests:
     * For empty area:
     *      Find: .*(Empty area:\s* center=\((\d+),(\d+)\), pixel=\((\d+),(\d+)\).*)
     *      Replace: {"\1", \2,\3,  mazes["4rooms2"], {\4,\5}},
     * For abrubt change:
     *      Find: .*(Abrubt change: center=\((\d+),(\d+)\)\->\((\d+),(\d+)\), (\d+)\->(\d+), pixel=\((\d+),(\d+)\).*)
     *      Replace: {"Test N1: \1", \2,\3,  mazes["4rooms2"], {\8,\9}}, {"Test N2: \1", \4,\5,  mazes["4rooms2"], {\8,\9}},
     */
    int x,y,row=0;
    bool display_moves = false;
    vector<vector<int>> pivot_map;
    
    struct Move {
        int dx;
        int dy;
        string name;
    };
    vector<Move> moves = {
        {1,  0, "RIGHT"},
        {-1, 0, "LEFT"},
        {0,  1, "UP"},
        {0, -1, "DOWN"},
    };

    mvprintw(row++, my_map->width()+4, "============ FAILED configurations: ===========");
    for (int cx=0; cx<my_map->width(); cx++)
        for (int cy=0; cy<my_map->height(); cy++) {
            mvprintw(my_map->height()+1, 0, "Current center point: (%d,%d)", cx, cy);
            my_map->add_walls(mazes[maze_index]);
            if (my_map->m[cx][cy]==MAP_CELL_EMPTY) {
                galaxy_big_bang(glx, cx, cy);
                if (display_all)
                    my_map->display();
                for (x=0; x<my_map->width(); x++)
                    for (y=0; y<my_map->height(); y++)
                        if (my_map->m[x][y]==MAP_CELL_EMPTY) {
                            mvprintw(row++, my_map->width()+4, "Empty area:    center=(%d,%d), pixel=(%d,%d)", cx,cy, x,y);
                            goto exit_loop_1;
                        }
              exit_loop_1:
                pivot_map= my_map->m;
                my_map->clean();
                
                if (distance_threshold!=0) {
                    //now try four directions < > ^ v
                    for (int k=0;k<4; k++) {
                        mvprintw(my_map->height()+2, 0, "Test point: (%d,%d)", cx+moves[k].dx, cy+moves[k].dy);
                        if (display_all)
                            refresh();
                        my_map->add_walls(mazes[maze_index]);
                        if (my_map->m[cx+moves[k].dx][cy+moves[k].dy]==MAP_CELL_EMPTY) {
                            galaxy_big_bang(glx, cx+moves[k].dx, cy+moves[k].dy);
                            if (display_moves && display_all)
                                my_map->display();
                            for (x=0; x<my_map->width(); x++)
                                for (y=0; y<my_map->height(); y++)
                                    if (my_map->m[x][y]>=0 && pivot_map[x][y]>=0 &&  fabs(my_map->m[x][y]-pivot_map[x][y])>distance_threshold) {
                                        mvprintw(row++, my_map->width()+4, "Abrubt change: center=(%d,%d)->(%d,%d), %d->%d, pixel=(%d,%d). Move=%s", 
                                                 cx,cy,  cx+moves[k].dx,cy+moves[k].dy,  pivot_map[x][y], my_map->m[x][y],  x,y, moves[k].name.c_str());
                                        goto exit_loop_2;
                                    }
                          exit_loop_2:
                            my_map->clean();
                        }
                    }
                }
            }
        }
    refresh();
}




void test_all_pixels_move_by_key(Galaxy& glx, const char*  maze_index, int distance_threshold) {
    /* Regexps to convert to static tests:
     * For empty area:
     *      Find: .*(Empty area:\s* center=\((\d+),(\d+)\), pixel=\((\d+),(\d+)\).*)
     *      Replace: {"\1", \2,\3,  mazes["4rooms2"], {\4,\5}},
     * For abrubt change:
     *      Find: .*(Abrubt change: center=\((\d+),(\d+)\)\->\((\d+),(\d+)\), (\d+)\->(\d+), pixel=\((\d+),(\d+)\).*)
     *      Replace: {"Test N1: \1", \2,\3,  mazes["4rooms2"], {\8,\9}}, {"Test N2: \1", \4,\5,  mazes["4rooms2"], {\8,\9}},
     */
    int x,y,row=0;
    bool display_moves = false;
    vector<vector<int>> pivot_map;
    
    struct Move {
        int dx;
        int dy;
        string name;
    };
    vector<Move> moves = {
        {1,  0, "RIGHT"},
        {-1, 0, "LEFT"},
        {0,  1, "UP"},
        {0, -1, "DOWN"},
    };

    mvprintw(row++, my_map->width()+4, "============ FAILED configurations: ===========");
    int cx=1, cy=1;
    while(true) {
            
            mvprintw(my_map->height()+1, 0, "Current center point: (%d,%d)", cx, cy);
            my_map->add_walls(mazes[maze_index]);
            if (my_map->m[cx][cy]==MAP_CELL_EMPTY) {
                galaxy_big_bang(glx, cx, cy);
                my_map->display();
                for (x=0; x<my_map->width(); x++)
                    for (y=0; y<my_map->height(); y++)
                        if (my_map->m[x][y]==MAP_CELL_EMPTY) {
                            mvprintw(row++, my_map->width()+4, "Empty area:    center=(%d,%d), pixel=(%d,%d)", cx,cy, x,y);
                            goto exit_loop_1;
                        }
              exit_loop_1:
                pivot_map= my_map->m;
                my_map->clean();
                
                if (distance_threshold!=0) {
                    //now try four directions < > ^ v
                    for (int k=0;k<4; k++) {
                        mvprintw(my_map->height()+2, 0, "Test point: (%d,%d)", cx+moves[k].dx, cy+moves[k].dy);
                        refresh();
                        my_map->add_walls(mazes[maze_index]);
                        if (my_map->m[cx+moves[k].dx][cy+moves[k].dy]==MAP_CELL_EMPTY) {
                            galaxy_big_bang(glx, cx+moves[k].dx, cy+moves[k].dy);
                            if (display_moves)
                                my_map->display();
                            for (x=0; x<my_map->width(); x++)
                                for (y=0; y<my_map->height(); y++)
                                    if (my_map->m[x][y]>=0 && pivot_map[x][y]>=0 &&  fabs(my_map->m[x][y]-pivot_map[x][y])>distance_threshold) {
                                        mvprintw(row++, my_map->width()+4, "Abrubt change: center=(%d,%d)->(%d,%d), %d->%d, pixel=(%d,%d). Move=%s", 
                                                 cx,cy,  cx+moves[k].dx,cy+moves[k].dy,  pivot_map[x][y], my_map->m[x][y],  x,y, moves[k].name.c_str());
                                        goto exit_loop_2;
                                    }
                          exit_loop_2:
                            my_map->clean();
                        }
                    }
                }
            }
        
        if (getch() == '\033') { // if the first value is esc
                getch(); // skip the [
                switch(getch()) { // the real value
                    case 'A':
                        if ( cy+1<my_map->height() )  cy++;
                        break;
                    case 'B':
                        if ( cy-1>=0 )  cy--;
                        break;
                    case 'C':
                        if ( cx+1<my_map->width() )  cx++;
                        break;
                    case 'D':
                        if ( cx-1>=0 )  cx--;
                        break;
                }
        }
    }
}




int main() {   

    sleep(2);
    initscr();
    
    my_map = new Map2D(W, H);
    init_pair(10, COLOR_RED,     COLOR_BLUE);
    
    #ifndef CWAVE_DEBUG
        Galaxy glx = galaxy_create(*my_map);
    #else
        CWaveTextDebugger dbg(*my_map);
        Galaxy glx = galaxy_create(*my_map, dbg);
    #endif
    
        my_map->display();
    
    int test_type=2;//
    
    switch(test_type) {
        case 0: test_manual_test_switch(glx, true, 58); break; //... test
        case 1: test_random_motion(glx, "4rooms2", false); break;
        case 2: test_all_pixels(glx, "4rooms2", 7, false); break;//maze_index, distance_threshold, display_all
        case 3: test_all_pixels(glx, "rooms_with_1px_doors", 7, true); break;//maze_index, distance_threshold, display_all
        case 4: test_all_pixels_move_by_key(glx, "rooms_with_1px_doors", 8); break;//maze_index, distance_threshold
        case 5: test_all_pixels_move_by_key(glx, "4rooms2", 10); break;//maze_index, distance_threshold
    }
    getch();
    refresh();
    endwin();

    return 0;

}
