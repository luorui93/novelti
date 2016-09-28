#include <stdlib.h>
#include <ncurses.h>
#include <low_throughput_hmi/fast_dist.h>
#include <low_throughput_hmi/map.h>


namespace low_throughput_hmi_cost {
    
class CWaveTextDebugger : public  CWaveDebugger {
public:
    Map2D& map_;
    bool galaxy_print_, star_print_, highlight_;
    
    CWaveTextDebugger(Map2D& map) :
            map_(map) {
        galaxy_print_=1;
        star_print_ = false;
        highlight_ = 1;
    }

    void map(Galaxy& glx){
        map_.display();
    };
    
    void highlight_boundary(Galaxy& glx, Star& star, Boundary& line) {
        int px, py;
        switch ((line).octant) {
            case 0: px=star.x+(line).x; py=star.y+(line).y; break;
            case 1: px=star.x+(line).y; py=star.y+(line).x; break;
            case 2: px=star.x-(line).y; py=star.y+(line).x; break;
            case 3: px=star.x-(line).x; py=star.y+(line).y; break;
            case 4: px=star.x-(line).x; py=star.y-(line).y; break;
            case 5: px=star.x-(line).y; py=star.y-(line).x; break;
            case 6: px=star.x+(line).y; py=star.y-(line).x; break;
            case 7: px=star.x+(line).x; py=star.y-(line).y; break;
        }
        //if (px>=0 && px<glx.map.width() && py>=0 && py<glx.map.height() && glx.map.m[px][py]!=MAP_CELL_OCCUPIED)
        highlight(glx, px,py, PIXEL_BOUNDARY);
    }
    
    void highlight(Galaxy& glx, int x, int y, PixelType type) {
        if (!highlight_ && type!=PIXEL_STAR && type!=PIXEL_STARDEAD)
            return;
        PixelColorPair color = 
            type==PIXEL_OVERLAP   ? PIXEL_COLOR_OVERLAP :
            type==PIXEL_WRONGDIST ? PIXEL_COLOR_WRONGDIST :
            type==PIXEL_EDGE      ? PIXEL_COLOR_EDGE :
            type==PIXEL_CANDIDATE ? PIXEL_COLOR_CANDIDATE :
            type==PIXEL_REGULAR   ? PIXEL_COLOR_REGULAR :
            type==PIXEL_STARDEAD  ? PIXEL_COLOR_REGULAR ://PIXEL_COLOR_STARDEAD :
            type==PIXEL_STAR      ? PIXEL_COLOR_STAR :
            type==PIXEL_BOUNDARY  ? PIXEL_COLOR_BOUNDARY :
                            PIXEL_COLOR_REGULAR;
        map_.highlight(x, y, color);
        map_.display();
    }
    
    void star_print(Galaxy& glx, Star& s) {
        if (!star_print_) return;
        int row=0, col=glx.map.width()+4;
        char walk_state[] = {'J','S','E','N'};
        mvprintw(row++, col, "== Star: (%d,%d) r=%d dist=%d origin=%d. ==   ", 
                 s.x, s.y, s.r, s.dist, s.walk.origin);
        mvprintw(row++, col, "walk: state=%c origin=%d prev=(oct=%d, %d,%d)  ", 
                 walk_state[s.walk.state], s.walk.origin, s.walk.prev_pt.oct, s.walk.prev_pt.x, s.walk.prev_pt.y);
        mvprintw(row++, col, "tmp_end: oct=%d d=(%d,%d) xy=(%d,%d), eps=%d   ",
            s.walk.tmp_end.octant, s.walk.tmp_end.dx, s.walk.tmp_end.dy, s.walk.tmp_end.x, s.walk.tmp_end.y, s.walk.tmp_end.eps );
        mvprintw(row++, col, "Beams:      ");
        for (forward_list<Beam>::iterator b=s.beams.begin();b!=s.beams.end();++b) {
            char mark = b==s.walk.curr ? 'C'  : (b==s.walk.prev ? 'P' : ' ');
            char lims = ' ';//b->start.is_limited ? 'L' : ' ';
            char lime = ' ';//b->end.is_limited ? 'L' : ' ';
            mvprintw(row++, col, "%c start: %c oct=%d d=(%d,%d) xy=(%d,%d), eps=%d   ",
                mark, lims, b->start.octant, b->start.dx, b->start.dy, b->start.x, b->start.y, b->start.eps);
            mvprintw(row++, col, "    end: %c oct=%d d=(%d,%d) xy=(%d,%d), eps=%d   ",
                lime, b->end.octant, b->end.dx, b->end.dy, b->end.x, b->end.y, b->end.eps);
            refresh();
        }
        mvprintw(row++, col, "======================       ");
        mvprintw(row++, col, "                             ");
        mvprintw(row++, col, "                             ");
        refresh();
    }

    void galaxy_print(Galaxy& glx) {
        if (!galaxy_print_) return;
        int row=0, col=glx.map.width()+21;//51;
        mvprintw(row++, col, "==== Galaxy w,h=(%d,%d) c=(%d,%d): ====   ", 
                 glx.map.width(), glx.map.height(), glx.map.width()/2, glx.map.height()/2);
        for (forward_list<Star>::iterator si=glx.stars.begin(); si!=glx.stars.end();++si) {
            char mark1 =  si==glx.ins ? 'I' : ' ';
            char mark2 =  si==glx.i ? 'i' : ' ';
            char mark3 =  si==glx.p ? 'p' : ' ';
            char empty = si->beams.empty() ? '=' : '>';
            mvprintw(row++, col, "%c%c%c  (%d,%d) r=%d dist=%d beams%c0 walk=?       ",
                mark1,mark2,mark3, si->x, si->y, si->r, si->dist, empty);
            refresh();
        }
        mvprintw(row++, col, "======================                ");
        mvprintw(row++, col, "dist=%d       ", glx.dist);
        mvprintw(row++, col, "                                           ");
        mvprintw(row++, col, "Candidates to grow:");
        for(std::vector<Candidate>::iterator cand = glx.candidates_to_grow.begin(); cand != glx.candidates_to_grow.end(); ++cand)
            mvprintw(row++, col, "    (%d,%d)              ", cand->x, cand->y);
        mvprintw(row++, col, "Candidates to add:");
        for(std::vector<Candidate>::iterator cand = glx.candidates_to_add.begin(); cand != glx.candidates_to_add.end(); ++cand)
            mvprintw(row++, col, "    (%d,%d)                ", cand->x, cand->y);
        
        mvprintw(row++, col, "                             ");
        mvprintw(row++, col, "                             ");        
        refresh();
    }
  
};
}