#include <low_throughput_hmi/arc_list.h>
#include <math.h>
#include <cmath>
#include <ncurses.h>

#include <stdlib.h> //REMOVE!

#define DEBUG 1

#define CHECK_MAP_BOUNDARIES 1

#define get_quadrant_by_cos_sin(cosine, sine) (((cosine)<-sqrt(2)/2 or (sine)<-sqrt(2)/2) <<1 | (fabs((cosine))<fabs(sine)))
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


#ifdef DEBUG
    #define DEBUG_SHOW_LINE_PIXEL( line )  \
                switch ((line).octant) {\
                    case 0: map_.highlight(cx_+(line).x, cy_+(line).y, 1); break;\
                    case 1: map_.highlight(cx_+(line).y, cy_+(line).x, 1); break;\
                    case 2: map_.highlight(cx_-(line).y, cy_+(line).x, 1); break;\
                    case 3: map_.highlight(cx_-(line).x, cy_+(line).y, 1); break;\
                    case 4: map_.highlight(cx_-(line).x, cy_-(line).y, 1); break;\
                    case 5: map_.highlight(cx_-(line).y, cy_-(line).x, 1); break;\
                    case 6: map_.highlight(cx_+(line).y, cy_-(line).x, 1); break;\
                    case 7: map_.highlight(cx_+(line).x, cy_-(line).y, 1); break;\
                }\
                map_.display();
#else
    #define DEBUG_SHOW_LINE_PIXEL(line)
#endif

namespace low_throughput_hmi_cost {

    ArcList::ArcList(Map2D &map, forward_list<ArcList> &focuses, int cx, int cy, int init_distance, double start_angle, double end_angle) :
        map_(map),
        focuses_(focuses)
    {
        cx_ = cx;
        cy_ = cy;
        init_dist_ = init_distance;
        radius_ = 0;
        cur_dist_ = init_dist_;
        ArcList::addSector(start_angle, end_angle);
        kk_=0;
    }
    
    void ArcList::addSector(double start_angle, double end_angle) {
        /*limits_.push_front(arc_range{{
                start_angle,      //start_angle
                cos(start_angle), // start_cos
                sin(start_angle), // start_sin
                LIMITED,          //start_status
                start_angle == 0.0 ? true : false // start_0
            }, {
                end_angle,      // end_angle
                cos(end_angle), // end_cos
                sin(end_angle), // end_sin 
                LIMITED,        // end_status
                end_angle >=6.3 ? true : false //end_360
            }});*/
    }
    
    ArcList::ArcList(Map2D &map, forward_list<ArcList> &focuses, int cx, int cy, int init_distance, int sx, int sy, int ex, int ey) :
        map_(map),
        focuses_(focuses)
    {
        cx_ = cx;
        cy_ = cy;
        init_dist_ = init_distance;
        radius_ = 0;
        cur_dist_ = init_dist_;
        ArcList::addSector(sx, sy, ex, ey);
        kk_=0;
    }
    

    
    void ArcList::addSector(int sx, int sy, int ex, int ey) {
        //F0 = pow(abs_x,2) + pow(abs_y,2) - pow(radius,2) + 1 - x + 2*y;
        limits_.push_front(Sector{
            ArcList::createBoundary(sx, sy),
            ArcList::createBoundary(ex, ey),
            STATE_JUST_STARTED,
            0,
            0
        });
    }    
    
    Boundary ArcList::createBoundary(int x, int y) {
        int xb, yb;
        char oct; //octant
        if (abs(x)>=abs(y))
            if (x>0)
                if (y>=0)   { xb=+x; yb=+y; oct=0;}
                else        { xb=+x; yb=-y; oct=7;}
            else
                if (y>0)    { xb=-x; yb=+y; oct=3;}
                else        { xb=-x; yb=-y; oct=4;}
        else
            if (y>=0)
                if (x>0)    { xb=+y; yb=+x; oct=1;}
                else        { xb=+y; yb=-x; oct=2;}
            else
                if (x>=0)   { xb=-y; yb=+x; oct=6;}
                else        { xb=-y; yb=-x; oct=5;}
                      //dx  dy  x  y  leps     eps octant   is_limited  is_origin 
        return Boundary{xb, yb, 0, 0, yb-xb/2, 0,  oct,     false,      false};
    }
    
    void ArcList::setCost0(int x, int y) {
        if (x>=0 && x<map_.w && y>=0 && y<map_.h)
            map_.m[x][y] = cur_dist_;
        map_.display();
        //getch();
    }

    void ArcList::processArcStartPixel(int px, int py) {
        /*char quadrant = get_quadrant_by_cos_sin(it_->start.sin, -(it_->start.cos));
        switch (quadrant) {
            case 0: px++; break;
            case 1: py++; break;
            case 2: px--; break;
            case 3: py--; break;
        }
        
        //if (map_.m[px][py]==MAP_CELL_EMPTY) map_.m[px][py]=MAP_CELL_CENTER;
        if (it_->start.status==LIMITED && map_.m[px][py]==MAP_CELL_EMPTY) {
            map_.m[px][py]=MAP_CELL_CENTER;
            focuses_.insert_after(where_to_insert_, ArcList(map_, focuses_, px, py, cur_dist_+1, 0.0, 7.0));
            ++where_to_insert_;
            it_->start.status=FREED;
        } else if (it_->start.status==FREED && map_.m[px][py]!=MAP_CELL_EMPTY) {
            it_->start.status=LIMITED;
        }
        //ArcList::printFocuses();*/
    }
    
    void ArcList::processArcEndPixel(int px, int py) {
        /*char quadrant = get_quadrant_by_cos_sin(-(it_->end.sin), it_->end.cos);
        switch (quadrant) {
            case 0: px++; break;
            case 1: py++; break;
            case 2: px--; break;
            case 3: py--; break;
        }
        //if (map_.m[px][py]==MAP_CELL_EMPTY) map_.m[px][py]=MAP_CELL_CENTER;
        if (it_->end.status==LIMITED && map_.m[px][py]==MAP_CELL_EMPTY) {
            map_.m[px][py]=MAP_CELL_CENTER;
            focuses_.insert_after(where_to_insert_, ArcList(map_, focuses_, px, py, cur_dist_+1, 0.0, 7.0));            
            ++where_to_insert_;
            it_->end.status=FREED;
        } else if (it_->end.status==FREED && map_.m[px][py]!=MAP_CELL_EMPTY) {
            it_->end.status=LIMITED;
        }
        //ArcList::printFocuses();*/
    }

    
    void ArcList::setCost(int x, int y) {
        // x and y are relative to the center (cx_, cy_)
        //if (!(x>=0 && x<map_.w && y>=0 && y<map_.h)) {
        int cur_cell_value, px=cx_+x, py=cy_+y;
        double d;
        
        if (px>=0 && px<map_.w && py>=0 && py<map_.h) {
            cur_cell_value =  map_.m[px][py];
            if (cur_cell_value == MAP_CELL_EMPTY) 
                map_.m[px][py] = cur_dist_;
            /*switch (arc_walk_state_) {
                case STATE_JUST_STARTED:
                    if (cur_cell_value==MAP_CELL_EMPTY) {
                        arc_walk_state_ = STATE_WAIT_FOR_END;
                        ArcList::processArcStartPixel(px, py);
                    } else {
                        arc_walk_state_ = STATE_WAIT_FOR_START;
                    }
                    break;
                case STATE_NEW_ARC_MAY_EXIST:
                    if (cur_cell_value==MAP_CELL_EMPTY) {
                        arc_range new_arc({{0.0, 0.0, 0.0, LIMITED, false}, {0.0, 0.0, 0.0, LIMITED, false}});
                        prev_it_ = it_;
                        it_ = limits_.insert_after(it_, new_arc);
                    }
                case STATE_WAIT_FOR_START:
                    if (cur_cell_value==MAP_CELL_EMPTY) {
                        arc_walk_state_=STATE_WAIT_FOR_END;
                        d = sqrt(x*x+y*y);
                        it_->start = orientation{atan2(y, x), x/d, y/d, LIMITED, false};
                    }
                    break;
                case STATE_WAIT_FOR_END:
                    if (cur_cell_value!=MAP_CELL_EMPTY) {
                        arc_walk_state_=STATE_NEW_ARC_MAY_EXIST;
                        d = sqrt(prev_x_*prev_x_+prev_y_*prev_y_);
                        it_->end = orientation{atan2(prev_y_, prev_x_), prev_x_/d, prev_y_/d, LIMITED, false};
                    }
            }
            prev_x_ = x; prev_y_ = y;*/
        }
        map_.display();
        //getch();
    }    
    
    void ArcList::drawAt(int radius) {/*
        //radius_=5;
        //radius_++;
        cur_dist_ = init_dist_+radius;
        //forward_list<arc_range>::iterator prev_it;
        orientation tmp_end;
        //bool origin_occupied =  map_.m[cx_+radius][cy_]!=MAP_CELL_EMPTY;
        for (it_ = limits_.begin(), prev_it_ = limits_.before_begin(); it_ != limits_.end(); prev_it_=it_, ++it_) {
            ArcList::printLimits();
            tmp_end=it_->end;
            //ArcList::printLimits();
            arc_walk_state_ = STATE_JUST_STARTED;
            //////////ArcList::basicWalk(radius, *it_);
            //ArcList::printLimits();
            switch (arc_walk_state_) {
                case STATE_WAIT_FOR_START: 
                    ArcList::printLimits();
                    it_=prev_it_;
                    limits_.erase_after(prev_it_); break;
                    ArcList::printLimits();
                case STATE_WAIT_FOR_END:
                    if (tmp_end.is_origin && map_.m[cx_+radius][cy_]<0) {
                        double d = sqrt(prev_x_*prev_x_+prev_y_*prev_y_);
                        it_->end = orientation{atan2(prev_y_, prev_x_), prev_x_/d, prev_y_/d, LIMITED, false};
                    } else {    
                        it_->end = tmp_end;
                        if (!tmp_end.is_origin)
                            ArcList::processArcEndPixel(cx_+prev_x_, cy_+prev_y_);
                    }    
            }
            ArcList::printLimits();
        }*/
    }

    void ArcList::grow(forward_list<ArcList>::iterator where_to_insert) {
        where_to_insert_ = where_to_insert;
        radius_++;
        ArcList::drawAt(radius_);
    }
    
    ArcList::~ArcList() {}
    
    void ArcList::printLimits() {
        int k = 2, col=82;
        mvprintw(k++, col, "========================");
        for (auto& lim: limits_) {
            ////mvprintw(k++, col, "    (%f, %f)    ", lim.start.angle, lim.end.angle);
            refresh();
        }
        mvprintw(k++, col, "==========%d==========", kk_++);
        refresh();
    }
    
    void ArcList::printFocuses() {
        int k = 2, col=115;
        mvprintw(k++, col, "========================");
        for (auto& foc: focuses_) {
            mvprintw(k++, col, "    (%d, %d)%c=> %d+%d    ", foc.cx_, foc.cy_, foc.limits_.empty()? 'E': ' ', foc.init_dist_, foc.radius_);
            refresh();
        }
        mvprintw(k++, col, "==========%d==========", kk_++);
        refresh();
    }
    
    
    //void ArcList::printSector(Sector& s) {}
    
    void ArcList::grow() {
        for (it_ = limits_.begin(), prev_it_ = limits_.before_begin(); it_ != limits_.end(); prev_it_=it_, ++it_) {
            processSectorGrowth(*it_);
        }
        radius_++;
        cur_dist_++;
    }
    
    #define is_origin(boundary) ((boundary).y==0 && (boundary).octant==0)
    void ArcList::processSectorGrowth(Sector& s) {
            tmp_end=it_->end;
            s.walk_state = WALK_JUST_STARTED;
            ArcList::growSector(s);
            switch (s.walk_state) {
                case WALK_WAIT_FOR_START: 
                    it_=prev_it_;
                    limits_.erase_after(prev_it_); break;
                case WALK_WAIT_FOR_END:
                    if (tmp_end.is_origin && map_.m[cx_+radius][cy_]<0) {
                        s.end = ArcList::createBoundary(s.prev.x,s.prev.y);
                    } else {    
                        it_->end = tmp_end;
                        if (!tmp_end.is_origin)
                            ArcList::processArcEndPixel(cx_+prev_x_, cy_+prev_y_);
                    }    
            }
            ArcList::printLimits();
    }
    
   
    #define combo(a,b) ((a)<<3 | (b))
    #define CELL_EMPTY 1
    void ArcList::walkPixel(int x, int y) {
        int px=cx_+x, py=cy_+y; //absolute coordinates
        #ifdef CHECK_MAP_BOUNDARIES
            if (!(px>=0 && px<map_.w && py>=0 && py<map_.h))
                return;
        #endif
        int cell = map_.m[px][py];
        #ifdef DEBUG
            //if (cur_cell_value>0) //check if the point has been previously visited
            //    map_.highlight(px, py, 3); //exit(2);//map_.highlight(cx_+x, cy_+y, 2); //        
            if (abs(sqrt(double(x*x+y*y))-radius_)>=0.5) //check the distance error
                map_.highlight(px, py, 2); //exit(2);//map_.highlight(cx_+x, cy_+y, 2); //
        #endif
        
        if (cell == MAP_CELL_EMPTY) 
            map_.m[px][py] = 2*dist_;
        switch (combo(s.walk, (cell==MAP_CELL_EMPTY)) {
            case (combo(WALK_JUST_STARTED,   CELL_EMPTY)): s.walk = WALK_WAIT_FOR_END;   break;// /*ArcList::processArcStartPixel(px, py);*/ break;
            case (combo(WALK_JUST_STARTED,  !CELL_EMPTY)): s.walk = WALK_WAIT_FOR_START; break;
            case (combo(WALK_NEW_ARC_MAYBE,  CELL_EMPTY)):
                arc_range new_arc({{0.0, 0.0, 0.0, LIMITED, false}, {0.0, 0.0, 0.0, LIMITED, false}});
                prev_it_ = it_;
                it_ = limits_.insert_after(it_, new_arc);
            case (combo(WALK_WAIT_FOR_START, CELL_EMPTY)): s.walk = WALK_WAIT_FOR_END;
                s.start = ArcList::createBoundary(x,y);
                break;
            case (combo(WALK_WAIT_FOR_END, !CELL_EMPTY)):  s.walk = WALK_NEW_ARC_MAYBE;
                s.end = ArcList::createBoundary(s.prev.x, s.prev.y);
        }
        s.prev.x=x; s.prev.y=y;
        #ifdef DEBUG
            map_.display();
        #endif
    }
    
    void ArcList::walkPixelNBP(int x, int y) {
        int px=cx_+x, py=cy_+y;
        if (map_.m[cx_+x][cy_+y]>0)
            map_.highlight(cx_+x, cy_+y, 3); //exit(2);//map_.highlight(cx_+x, cy_+y, 2); //
        //if (!(px>=0 && px<map_.w && py>=0 && py<map_.h)) return;        
        map_.m[cx_+x][cy_+y] = 2*cur_dist_+1;
        map_.display();
    }    
    
    bool ArcList::lineAddPixel(Boundary& line) { //returns true if y is incremented
        bool incremented=false;
        line.x++;
        if (line.leps>0) {
            line.leps -= line.dx;
            line.y++;
            incremented = true;
        }
        line.leps += line.dy;
        DEBUG_SHOW_LINE_PIXEL(line);
        return incremented;
    }
    
    void ArcList::calculateArcTip(Boundary& b, ArcTip& output_tip) {
        //eps = x^2+y^2-r^2
        if (b.eps-b.x>=0 && radius_!=0) {    //Start pixel is off the boundary line
            output_tip.offline = true;
        } else {                            //Start pixel is on the boundary line
            b.eps += 2*b.x+1;               // eps -> eps(x+1,y,r)
            if (lineAddPixel(b)) {
                b.eps += 2*(b.y-radius_-1); // eps -> eps(x+1,y+1,r+1)
                return;
            } 
        }

        //Check condition for non-Bresenham point (NBP)
        b.eps += 2*(b.x-radius_);           //eps -> eps(x+1,y,r+1) 
        if (b.eps-b.x<=0 || (output_tip.offline && radius_==0)) {   //Non-Bresenham point
            output_tip.nbp = true;
            if (lineAddPixel(b))
                b.eps += 2*b.y-1;           // eps -> eps(x+1,y+1,r+1)
        } else {                            //Regular (Bresenham) point
            b.eps -= 2*b.x+1;               // eps -> eps(x,y,r+1)
        }
    }
    
    
    void ArcList::growSector(Sector& s) {
        //calculate arc start and end tips
        struct ArcTip start_tip = {s.start.x, s.start.y, s.start.eps, false, false, s.start.octant};
        ArcList::calculateArcTip(s.start, start_tip);
        struct ArcTip end_tip = {s.end.x, s.end.y, s.end.eps, false, false, s.end.octant};
        ArcList::calculateArcTip(s.end, end_tip);
        
        ArcList::arcWalk(start_tip, end_tip);
    }
    
    #define nbp_octant_1   (x+1),  y
    #define nbp_octant_3  -y    ,  (x+1)
    #define nbp_octant_5  -(x+1), -y    
    #define nbp_octant_7   y    , -(x+1)
    
    #define end_nbp_octant_1   x    ,  (y+1)
    #define end_nbp_octant_3  -(y+1),  x
    #define end_nbp_octant_5  -x    , -(y+1)
    #define end_nbp_octant_7   (y+1), -x

    #define visitPixel(x,y) ArcList::visitPixel( (x), (y) )
    #define visitPixelNBP(x,y) ArcList::visitPixelNBP( (x), (y) )            
    
    #define octant_even_loop(oct, xp, yp, exit_condition)\
        while ( exit_condition ) {\
            visitPixel( (xp), (yp) );\
            y++;\
            if (F>=0) {\
                if (F+2*(x-radius_)-1<0)\
                    ArcList::walkPixelNBP( (xp), (yp) );\
                F += -2*(--x);\
            }\
            F += 2*y+1;\
        }

    #define octant_odd_loop(oct, xp, yp, exit_condition) \
        while ( exit_condition ) {\
            visitPixel( (xp), (yp) );\
            x--;\
            if (F<0) {\
                F += 2*(++y);\
                if (F + 2*(x-radius_)<0)\
                    visitPixelNBP( nbp_octant_##oct );\
            }\
            F += -2*x+1; \
        }

    #define octant_even(oct, xp, yp)\
        if (end.octant==(oct)) {\
            octant_even_loop( oct, (xp), (yp), y<end1 );\
            if (x==end.x) \
                visitPixel( (xp), (yp) );\
            return;\
        } \
        octant_even_loop( oct, (xp), (yp), y<x ); \
        F -=(x+y); 
        
    #define octant_odd(oct, xp, yp)\
        if (end.octant==(oct)) {\
            octant_odd_loop( oct, (xp), (yp), x>end1 );\
            /*if (y==0 && (oct)==7 && origin_visited) */\
            if (x>=end1) {/* Remove this IF-block if end boundary for odd octants is not needed*/\
                visitPixel( (xp), (yp) );\
                if (end.nbp) \
                    visitPixelNBP( end_nbp_octant_##oct );\
            }\
            return;\
        }\
        octant_odd_loop( oct, (xp), (yp), x>0 );\
        x=radius_; y=0; F =1-radius_; 
    
    void ArcList::arcWalk(Sector& sector, const ArcTip& start, const ArcTip& end) {
        int tmp, x=start.x, y=start.y, end1 = end.y, F=start.eps+1;
        bool start_nbp;        
        if (start.octant%2==0) {
            F -= x-2*y;
            if (start.offline) 
                F-= 2*(--x);
            start_nbp = start.nbp;
        } else {
            F += x-2*y;
            if (start.offline)
                F-= 2*(--y)-1; 
            int tmp=x; x=y; y=tmp; //swap x and y
            start_nbp = start.nbp && start.offline;
        }
        if (start_nbp)
            switch (start.octant) {
                case 0: case 1: visitPixelNBP( nbp_octant_1 ); break;
                case 2: case 3: visitPixelNBP( nbp_octant_3 ); break;
                case 4: case 5: visitPixelNBP( nbp_octant_5 ); break;
                case 6: case 7: visitPixelNBP( nbp_octant_7 ); break;
            }
        switch(start.octant) {
            case 0: octant_even(0,  x,  y);
            case 1: octant_odd (1,  x,  y);
            case 2: octant_even(2, -y,  x);
            case 3: octant_odd (3, -y,  x);
            case 4: octant_even(4, -x, -y);
            case 5: octant_odd (5, -x, -y);
            case 6: octant_even(6,  y, -x);
            case 7: octant_odd (7,  y, -x);            
        }
    };

    
    void ArcList::updateEndBoundary(Boundary& b, int end_status) {
        /* Modifies b */
        bool on_boundary = !(end_status & BOUNDARY_STATUS_OFF_BOUNDARY);
        bool nbp = end_status & BOUNDARY_STATUS_NON_BRES_POINT;
        if (on_boundary && nbp) //on boundary,  nbp
            if (lineAddPixel(b))
                return;
        if (on_boundary || nbp)  //on boundary OR nbp
            lineAddPixel(b);
    }
    
    //arc_walker(ArcList::basicWalk, ArcList::setCost)}
    /*void ArcList::basicWalk(int radius, arc_range& range) {
        //calculate start and end points
        int x = int(round(radius* range.start.cos));
        int y = int(round(radius* range.start.sin));
        int end_x, end_y;
        if (range.end.is_origin) { //full circle
            end_x=radius; 
            end_y=-1;
        } else {
            end_x = int(round(radius* range.end.cos));
            end_y = int(round(radius* range.end.sin)); 
        }
        //if (end_y==0 && end_x>0) end_y--; // for full circles or any arc ending in 0rad

        //calculate start octant
        int abs_x=abs(x), abs_y=abs(y);
        short start_octant = abs_x>=abs_y
            ? x>0 // abs_x>=abs_y
                ? (y>=0 ? 0 : 7)
                : (y>0  ? 3 : 4)
            : y>=0 // abs_x<abs_y
                ? (x>0  ? 1 : 2)
                : (x>=0 ? 6 : 5);
        if (x==y) start_octant++;
        
        //calculate end octant    
        int abs_end_x=abs(end_x), abs_end_y=abs(end_y);
        short end_octant = abs_end_x>=abs_end_y
            ? end_x>0 // abs_x>=abs_y
                ? (end_y>=0 ? 0 : 7)
                : (end_y>0  ? 3 : 4)
            : end_y>=0 // abs_x<abs_y
                ? (end_x>0  ? 1 : 2)
                : (end_x>=0 ? 6 : 5);
        if (end_x==end_y) end_octant++;

        if (radius==1) { //special case: in this case octants 1,3,5,7 don't exist
            int start_quadrant = get_quadrant_by_cos_sin(range.start.cos, range.start.sin);
            int end_quadrant = range.end.is_origin ? 3 :
                get_quadrant_by_cos_sin(range.end.cos, range.end.sin);
            switch (start_quadrant) {
                case 0: ArcList::setCost( +1,  +0); if (end_quadrant==0) return;
                case 1: ArcList::setCost( +0,  +1); if (end_quadrant==1) return;
                case 2: ArcList::setCost( -1,  +0); if (end_quadrant==2) return;
                case 3: ArcList::setCost( +0,  -1); if (end_quadrant==3) return;
            }
            /*switch (start_octant) {
                case 0: ArcList::setCost( +1,  +0); if (end_octant==0) return;
                case 1: ArcList::setCost( +1,  +1); if (end_octant==1) return;
                case 2: ArcList::setCost( +0,  +1); if (end_octant==2) return;
                case 3: ArcList::setCost( -1,  +1); if (end_octant==3) return;
                case 4: ArcList::setCost( -1,  +0); if (end_octant==4) return;
                case 5: ArcList::setCost( -1,  -1); if (end_octant==5) return;
                case 6: ArcList::setCost( -0,  -1); if (end_octant==6) return;
                case 7: ArcList::setCost( +1,  -1); if (end_octant==7) return;            
            }*  /
            return;
        }

    
        //calculate initial value for F (decision over 2)
        int F = pow(abs_x,2) + pow(abs_y,2) - pow(radius,2) + 1;
        switch (start_octant) {
            case 0: F += -x+2*y; break;
            case 1: F += -2*x+y; break;
            case 2: F += -2*x-y; break;
            case 3: F += -x-2*y; break;
            case 4: F +=  x-2*y; break;
            case 5: F +=  2*x-y; break;
            case 6: F +=  2*x+y; break;
            case 7: F +=  x+2*y; break;
        }
        
        //set the limit for the end point
        //int end = (abs_end_x>abs_end_y) ? end_y : end_x;//s[8] = {0};
        //ends[end_octant] = (abs_end_x>abs_end_y) ? end_y : end_x;
        
        //mvprintw(+3, , "sq=%d, eq=%d", start_octant, end_octant);
        loop_again:
        switch (start_octant) {
            case 0:
                do {
                    ArcList::setCost( +x,  +y);
                    y++;
                    F +=  F>=0 ?  2*(y-(--x))+1 : 2*y+1; 
                    if (end_octant==0 && y>=end_y) { 
                        if (y==end_y) ArcList::setCost( +x,  +y);
                        return;
                    }
                } while (y<x);
                F += -x-y;//-2*x;
            case 1:
                do {
                    ArcList::setCost( +x,  +y);
                    x--;
                    F +=  F<0 ?  2*((++y)-x)+1 : -2*x+1; 
                    if (end_octant==1 && x<=end_x) { 
                        if (x==end_x) ArcList::setCost( +x,  +y);
                        return;
                    }
                } while (x!=0);
                F = 1-radius;
            case 2:
                do {
                    ArcList::setCost( +x,  +y);
                    x--;
                    F +=  F>=0 ? -2*(x+(--y))+1 : -2*x+1; 
                    if (end_octant==2 && x<=end_x) { 
                        if (x==end_x) ArcList::setCost( +x,  +y);
                        return;
                    }
                } while (-x<y);
                F += x-y;//-2*y;
            case 3:
                do {
                    ArcList::setCost( +x,  +y);
                    y--;
                    F +=  F<0 ? -2*((--x)+y)+1  : -2*y+1; 
                    if (end_octant==3 && y<=end_y) { 
                        if (y==end_y) ArcList::setCost( +x,  +y);
                        return;
                    }
                } while (y!=0);
                F = 1-radius;  
            case 4:
                do {
                    ArcList::setCost( +x,  +y);
                    y--;
                    F +=  F>=0 ?  2*((++x)-y)+1 : -2*y+1; 
                    if (end_octant==4 && y<=end_y) { 
                        if (y==end_y) ArcList::setCost( +x,  +y);
                        return;
                    }
                } while (y>x);
                F += x+y;//+2*x;
            case 5:
                do {
                    ArcList::setCost( +x,  +y);
                    x++;
                    F +=  F<0 ?  2*(x-(--y))+1 : 2*x+1; 
                if (end_octant==5 && x>=end_x) { 
                        if (x==end_x) ArcList::setCost( +x,  +y);
                        return;
                    }
                } while (x!=0);
                F = 1-radius;
            case 6:
                do {
                    ArcList::setCost( +x,  +y);
                    x++;
                    F +=  F>=0 ?  2*(x+(++y))+1 : 2*x+1;
                    if (end_octant==6 && x>=end_x) { 
                        if (x==end_x) ArcList::setCost( +x,  +y);
                        return;
                    }
                    //mvprintw(+3, , "61: (%d,%d) ends6=%d", x,y, ends[6]);
                } while (x<-y);
                F += -x+y;//+2*y;
            case 7:
                do {
                    ArcList::setCost( +x,  +y);
                    y++;
                    F +=  F<0 ?  2*((++x)+y)+1 : 2*y+1;
                    if (end_octant==7 && y>=end_y) { 
                        if (y==end_y) ArcList::setCost( +x,  +y);
                        return;
                    }
                } while (y!=0);
                //if (end_octant==7) break;
                //F = F+2*y;
        }
        goto loop_again;
        last_point:
            if (x==end_x && y==end_y)
                ArcList::setCost( +x,  +y);
    }*/
}
