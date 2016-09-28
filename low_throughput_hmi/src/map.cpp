#include <ncurses.h>  // Documentation for NCurses: http://www.tldp.org/HOWTO/html_single/NCURSES-Programming-HOWTO
//#include <math.h>  
#include <time.h>       /* time */
#include <vector>
#include <cstdlib>
#include <low_throughput_hmi/map.h>


#define DrawPixel(x, y, c) mvaddch(HEIGHT-1-(y), x, c); //refresh();//getch();

using namespace std;

namespace low_throughput_hmi_cost {

    Map2D::Map2D(int width, int height){
        w = width;
        h = height;
        m=vector<vector<int>>(w, vector<int>(h, MAP_CELL_EMPTY));
        color=vector<vector<int>>(w, vector<int>(h, 0));
        
        start_color();
        init_color(COLOR_YELLOW, 1000, 1000, 1000);
                                        //font        background
        init_pair(PIXEL_COLOR_REGULAR,   COLOR_CYAN,   COLOR_BLACK);
      //init_pair(PIXEL_COLOR_OCCUPIED,  COLOR_WHITE,  COLOR_WHITE);
        init_pair(PIXEL_COLOR_OCCUPIED,  COLOR_YELLOW, COLOR_YELLOW);
        init_pair(PIXEL_COLOR_BOUNDARY,  COLOR_RED,    COLOR_BLACK);
        init_pair(PIXEL_COLOR_WRONGDIST, COLOR_WHITE,  COLOR_RED);
        init_pair(PIXEL_COLOR_OVERLAP,   COLOR_BLUE,   COLOR_BLACK);
        init_pair(PIXEL_COLOR_CANDIDATE, COLOR_WHITE,  COLOR_BLUE);
        init_pair(PIXEL_COLOR_STAR,      COLOR_RED,    COLOR_WHITE);
        init_pair(PIXEL_COLOR_STARDEAD,  COLOR_BLACK,  COLOR_WHITE);
        init_pair(PIXEL_COLOR_EDGE,      COLOR_BLACK,  COLOR_YELLOW);
        
        init_pair(PIXEL_COLOR_BORDER,    COLOR_YELLOW, COLOR_RED);
        init_pair(PIXEL_COLOR_REGION+0,  COLOR_WHITE,  COLOR_MAGENTA);
        init_pair(PIXEL_COLOR_REGION+1,  COLOR_BLACK,  COLOR_RED);
        init_pair(PIXEL_COLOR_REGION+2,  COLOR_WHITE,  COLOR_BLUE);
        init_pair(PIXEL_COLOR_REGION+3,  COLOR_BLACK,  COLOR_CYAN);
        init_pair(PIXEL_COLOR_REGION+4,  COLOR_BLACK,  COLOR_WHITE);
        
        //best pose finder
        init_pair(PIXEL_COLOR_UNREACH,            COLOR_YELLOW,  COLOR_YELLOW);
        init_pair(PIXEL_COLOR_REACH_NOT_VISITED,  COLOR_CYAN,  COLOR_BLACK);
        init_pair(PIXEL_COLOR_REACH_VISITED,      COLOR_CYAN,  COLOR_BLUE);

        //init_pair(PIXEL_COLOR_CANDIDATE, COLOR_YELLOW, COLOR_YELLOW);
        int q;
        for (int x=0;x<w;x++) {
            int q = x%10;
            if (q==0)
                mvprintw(h, x, " ");
            else 
                mvprintw(h, x, "%d", q);
        }
        for (int y=0;y<h;y++) {
            mvprintw(h-1-y, w+1, "%d", y);
        }
        /*mvprintw(0, 0, ">");
        getch();
        mvprintw(0, 0, "   m[0][0]=%d     ", get(0,0));
        getch();*/
    }
    
    int Map2D::get(int x, int y) {
        return m[x][y];
    }
    
    void Map2D::set(int x, int y, int val) {
        m[x][y]=val;
    }
    
    int Map2D::width() {
        return w;
    }
    
    int Map2D::height() {
        return h;
    }
        
    void Map2D::display(int row, int col) {
        int p;
        char c;
        for (int x=0;x<w;x++) {
            for (int y=0;y<h;y++) {
                p = get(x,y);
                if (p==MAP_CELL_EMPTY) c='.';//continue;
                else if (p==MAP_CELL_OCCUPIED) c='X';
                else if (p==MAP_CELL_CENTER) c='+';
                else c=char((p % 10)+48);
                /*else if (p<=9) c=char(p+48);
                else if (p<=35) c=char(p+(97-10));
                else if (p<=61) c=char(p+(65-36));
                else if (p<=76) c=char(p+(33-62));
                else if (p<=83) c=char(p+(58-77));
                else if (p<=89) c=char(p+(91-84));
                else if (p<=93) c=char(p+(123-90));//*/
                if (c=='0') c='`';
                if (p==MAP_CELL_OCCUPIED) {
                    attron(COLOR_PAIR(PIXEL_COLOR_OCCUPIED));
                    mvaddch(row+h-1-y, col+x, c);
                    attroff(COLOR_PAIR(PIXEL_COLOR_OCCUPIED));
                } else {
                    attron(COLOR_PAIR(color[x][y]));
                    mvaddch(row+h-1-y, col+x, c);
                    attroff(COLOR_PAIR(color[x][y]));
                }
                //DrawPixel( x,  y, c);
            }
        
        }
        refresh();
    }
    
    void Map2D::display() {
        display(0,0);
    }
        
    void Map2D::highlight(int x, int y, int color_pair_id) {
        if (x>=0 && x<w && y>=0 && y<h)
            color[x][y]=color_pair_id;
    }
    
    void Map2D::add_obstacles(int number_of_pixels) {
        int x,y;    

        srand (time(NULL));
        for (int k=0;k<number_of_pixels;k++) {
            x = rand() % w;
            y = rand() % h;
            //mvprintw(0, 0, "      x=%d, y=%d     ", x, y);
            //getch();
            set(x,y, MAP_CELL_OCCUPIED);
        }
    }
    
    void Map2D::add_wall(int x0, int y0, int l) {
        if (l>0)
            for (int x=x0; x<x0+l; x++) {
                set(x,y0, MAP_CELL_OCCUPIED);
            }
        else
            for (int y=y0; y<y0-l; y++) {
                set(x0,y, MAP_CELL_OCCUPIED);
            }
    }
    
    void Map2D::add_walls(vector<Wall>& walls) {
        for (int w=0;w<walls.size();w++) {
            add_wall(int(walls[w].x), int(walls[w].y), int(walls[w].l));
        }
    }
    
    void Map2D::add_from_bits(bool** cell_traversable) {
        for (int y=0; y<h; y++)
            for (int x=0; x<w; x++)
                if (cell_traversable[x][y])
                    set(x,y,MAP_CELL_EMPTY);
                else
                    set(x,y,MAP_CELL_OCCUPIED);
    }
    
    void Map2D::clean() {
        for (int x=0;x<w;x++)
            for (int y=0;y<h;y++) {
                set(x,y, MAP_CELL_EMPTY);
                color[x][y]=0;
            }
    }
    
    void Map2D::clean_dist() {
        for (int x=0;x<w;x++)
            for (int y=0;y<h;y++) {
                if (get(x,y)>=0)
                    set(x,y, MAP_CELL_EMPTY);
            }
    }
    
    Map2D::~Map2D(){};
}