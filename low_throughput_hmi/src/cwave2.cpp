#include <low_throughput_hmi/fast_dist.h>
#include <cmath>


namespace cwave2 {
    
//Point
    Point::Point(int x0, int y0) {
        x = x0;
        y = y0;
    }
    
    Point::Point(OctPoint& p, Point& c) {
        switch (p.oct) {
            case 0: return { c.x+p.x, c.y+p.y };
            case 1: return { c.x+p.y, c.y+p.x };
            case 2: return { c.x-p.y, c.y+p.x };
            case 3: return { c.x-p.x, c.y+p.y };
            case 4: return { c.x-p.x, c.y-p.y };
            case 5: return { c.x-p.y, c.y-p.x };
            case 6: return { c.x+p.y, c.y-p.x };
            case 7: return { c.x+p.x, c.y-p.y };
        }
    }

//OctPoint
    OctPoint::OctPoint(char oct0, int x0, int y0, int F0, bool reg0, bool nbp0) {
        oct = oct0;
        x   = x0;
        y   = y0;
        F   = F0;
        reg = reg0;
        nbp = nbp0;
    }
    
    
//Boundary
    Boundary::Boundary(Point p, int eps0) {
        if (abs(p.x)>=abs(p.y))
            if (p.x>0)
                if (p.y>=0) { dx=+p.x; dy=+p.y; oct=0+(p.x==p.y);}
                else        { dx=+p.x; dy=-p.y; oct=7;}
            else
                if (p.y>0)  { dx=-p.x; dy=+p.y; oct=3;}
                else        { dx=-p.x; dy=-p.y; oct=4+(p.x==p.y);}
        else
            if (p.y>=0)
                if (p.x>0)  { dx=+p.y; dy=+p.x; oct=1;}
                else        { dx=+p.y; dy=-p.x; oct=2;}
            else
                if (p.x>=0) { dx=-p.y; dy=+p.x; oct=6;}
                else        { dx=-p.y; dy=-p.x; oct=5;}
        x=0;
        y=0;
        leps = y-x/2;
        eps  = eps0;
        tip = {0};
    }

    Boundary::Boundary(OctPoint& op, int radius, CWave2& cw) {
        oct = op.oct;        
        dx = op.x;
        dy = op.y;
        x  = op.x;
        y  = op.y;
        eps = op.F-1 + (op.oct%2 ?  2*op.y-x : x-2*op.y);
        leps = y-x/2
        calcTip(radius, cw);
    }
    
    void Boundary::calcTip(int radius,  CWave2& cw) {
        tip = {x, y, eps, false, false};
        //eps = x^2+y^2-r^2
        if (eps-x>=0) {    //Start pixel is off the boundary line
            tip.offline = true;
        } else {                        //Start pixel is on the boundary line
            eps += 2*x+1;               // eps -> eps(x+1,y,r)
            if (addPixel(cw)) {
                eps += 2*(y-radius-1);  // eps -> eps(x+1,y+1,r+1)
                return;
            }
        }

        //Check condition for non-Bresenham point (NBP)
        eps += 2*(x-radius);           //eps -> eps(x+1,y,r+1) 
        if (eps-x<=0) {   //Non-Bresenham point
            tip.nbp = true;
            if (addPixel(cw))
                eps += 2*y-1;           // eps -> eps(x+1,y+1,r+1)
        } else {                        //Regular (Bresenham) point
            eps -= x+1;                 // eps -> eps(x,y,r+1)
        }
    }
    
    bool Boundary::addPixel(CWave2& cw) { //returns true if y is incremented
        bool incremented=false;
        line.x++;
        if (line.leps>0) {
            line.leps -= line.dx;
            line.y++;
            incremented = true;
        }
        line.leps += line.dy;
        CWAVE_DBG(highlight_boundary);
        return incremented;
    }


//Beam
    Beam::Beam() { //full circle beam
        start = Boundary({2,0}, 0);
        end   = Boundary({2,0}, 0);
        end.oct = 7;
    }
    
    
//Star
    Star::Star(const Point& c0, int init_dist) {
        beams_.push_front(Beam());
        c = c0;
        r = 1;
        dist = init_dist+2
           //state              inclusive origin           beam           prev                  prev_pt     tmp_end;
        w = {WALK_JUST_STARTED, false,    ORIGIN_AT_START, beams.begin(), beams.before_begin(), {0},        {0}};
    }
    
    bool Star::empty() { return beams_.empty();}
    
    void Star::addBeam(Beam beam) {
        w.prev = w.beam;
        w.beam = beams_.insert_after(w.beam, beam);
    }
    
    void Star::deleteCurBeam() {
        w.beam = w.prev;
        beams_.erase_after(w.prev); 
    }
    
    #define is_origin(boundary) ((boundary).y==0 && (boundary).octant==0)
    void Star::grow(CWave2& cw) {
        w.origin = (!beams_.empty() && is_origin(beams_.begin()->start)) ? ORIGIN_AT_START : ORIGIN_NOT_AT_START;
        for (w.beam  = beams_.begin(), w.prev = beams_.before_begin(); 
                w.beam != beams_.end(); 
                w.prev  = w.beam, ++w.beam) {
            growCurBeam();
        }
        r_++;
        dist+=2;
    }

    #define octant_even_loop(exit_condition, condition2)\
        w.p.reg = true;\
        while ( exit_condition ) {\
            /*should be: F = x*x+y*y-star.r*star.r-x+2*y+1;*/\
            if (visitPair(cw)) return;\
            w.p.nbp = (w.p.F>=0) && ((w.p.F-=2*(--w.p.x)) + 2*(2*w.p.x-r)+1 < 0);\
            w.p.F += 2*(++w.p.y)+1;\
        }\
        w.p.reg = (condition2);\
        if (visitPair(cw)) return;

    #define octant_odd_loop(exit_condition) \
        w.p.reg = true;\
        while ( exit_condition ) {\
            x = w.p.x;  F = w.p.F; /*should be: F = x*x+y*y-star.r*star.r+x-2*y+1;*/\
            w.p.nbp = (F<0) && ( (F+=2*(++x)) + 2*(w.p.y-r-1) < 0 );\
            if (visitPair(cw)) return;\
            w.p.x = x;  w.p.F = F;\
            w.p.F += -2*(--w.p.y)+1; \
        }

    #define octant_even(oct)\
        w.p.oct = (oct);\
        if (end_octant==(oct)) {\
            octant_even_loop( w.p.y<end_tip_y, w.p.x==end_tip_x );\
            return;\
        } \
        octant_even_loop( w.p.y<w.p.x, false );\
        if (w.p.x!=w.p.y)\
            {w.p.x++;w.p.y--;};\
        w.p.F -= (w.p.x+w.p.y);

    #define octant_odd(oct)\
        w.p.oct = (oct);\
        w.p.nbp = false;\
        if (end_octant==(oct)) {\
            octant_odd_loop( w.p.y>end_tip_y );\
            w.p.reg = w.p.y>=end_tip_y; \
            w.p.nbp = end_tip_nbp;\
            if (w.p.reg) visitPair(cw)); \
            return;\
        }\
        octant_odd_loop( w.p.y>0 );\
        w.p.x=r; w.p.y=0; w.p.F=1-w.p.x; 
        
    void Star::arcWalk(CWave2& cw) {
                     //oct                x                    y
        w.p = OctPoint(w.beam->start.oct, w.beam->start.tip.x, w.beam->start.tip.y, 
                     //F                        reg    nbp 
                       w.beam->start.tip.eps+1, false, w.beam->start.tip.nbp);
        int x_, F_, 
            end_tip_x    = w.beam->end.tip.x,
            end_tip_y    = w.beam->end.tip.y, 
            end_octant   = w.beam->end.oct;
        bool end_tip_nbp = w.beam->end.tip.nbp;
              
        if (w.beam->start.oct%2==0) {
            w.p.F -= w.p.x-2*w.p.y;
            if (w.beam->start.tip.offline) 
                w.p.F -= 2*(--w.p.x);
        } else {
            w.p.F += w.p.x-2*w.p.y;
            if (w.beam->start.tip.offline) {
                w.p.F -= 2*(w.p.x--);
                if (visitPair(cw)) return;
                w.p.F += 2*((++w.p.x)-(--w.p.y))+1; 
            }
        }
        switch(w.beam->start.oct) {
            case 0: octant_even(0);
            case 1: octant_odd (1);
            case 2: octant_even(2);
            case 3: octant_odd (3);
            case 4: octant_even(4);
            case 5: octant_odd (5);
            case 6: octant_even(6);
            case 7: octant_odd (7);
        }/**/
    };
    
    bool visitPair(CWave2& cw) {
        //returns true if beam_arc_walk should return
        int cost = MAP_CELL_EMPTY;
        bool with_nbp=false;
        if (w.p.reg) { //regular point present
            if (p.oct==7 && p.y==0 && w.origin==ORIGIN_AT_START) {
                w.origin=ORIGIN_CONNECTED; 
                return false;
            }
            cost = cw->markPixel(Point(w.p,c), false);
        } else if (!w.p.nbp)
            return false;
        if (cost!=MAP_CELL_OCCUPIED && w.p.nbp) { 
            F+=2*((x++) +(oct%2));
            cost = markPixel(Point(w.p,c), true);
            with_nbp=w.p.reg;
        }
        return actOnPixel(glx, star, beam, oct, x, y, F, cost, with_nbp);
    }
    
    bool actOnPixel(int cost) {
        // returns true if beam_arc_walk should finish
        int action = 0;
        bool inclusive = (w.state== WALK_JUST_STARTED ? (cost>=0): w.inclusive);
        w.inclusive = (cost>=0);
        if ((cost==MAP_CELL_EMPTY && !inclusive) || cost>=0)
            w.prev_pt = {oct, x, y, F, with_nbp};
        if (cost==MAP_CELL_EMPTY) {
            switch(w.state) {
                case WALK_JUST_STARTED:
                    if (! w.beam->start.tip.offline || w.beam->start.tip.nbp)                        
                        onBoundary(w.prev_pt, IS_START);
                    break;
                case WALK_WAIT_FOR_START: 
                    action=1;
                    break;
                case WALK_NEW_ARC_MAYBE: 
                    action=2;
                    break;
            }
            w.state =  WALK_WAIT_FOR_END;
        } else { //occupied or previously visited pixel
            switch(w.state) {
                case WALK_JUST_STARTED:     w.state =  WALK_WAIT_FOR_START;
                    if (with_nbp)
                        onBoundary(w.p, IS_START, cw);
                    break;
                case WALK_WAIT_FOR_START:
                    if (inclusive & cost==MAP_CELL_OCCUPIED)        //test 13 => test 15
                        onBoundary(w.prev_pt, IS_END, cw);      //test 13
                    break;
                case WALK_WAIT_FOR_END:     w.state =  WALK_NEW_ARC_MAYBE;
                    action=3;
                    break;
            }
        }
        if (action>0) {
            Boundary b = Boundary(w.prev_pt, star.r, cw);
            if      (action==1) {
                w.beam->start = b; 
            } else if (action==2){
                addBeam({b, w.tmp_end}); 
            } else {
                w.tmp_end = w.beam->end; //TODO is this a copy?
                w.beam->end = b; 
            }
            onBoundary(w.prev_pt, action!=3, cw); 
        }
        
        if ((cost>=-1 && inclusive) || (cost==MAP_CELL_OCCUPIED && with_nbp))
            w.prev_pt = w.p;
        CWAVE_DBG(star_print, star);
        CWAVE_DBG(map);

        return false;
    }
    
    
    void growCurBeam(CWave2& cw) {
        w.state = WALK_JUST_STARTED;
        w.inclusive = false;
        
        Beam *bm = w.beam;
        //calculate arc start and end tips
        bm.start->calcTip(r,  cw, this);
        bm.end  ->calcTip(r,  cw, this);
        arcWalk(cw);
            
        switch (w.state) {
            case WALK_JUST_STARTED: 
            case WALK_WAIT_FOR_START:
                if (w.inclusive)
                     onBoundary(w.prev_pt, IS_END, cw);
                deleteCurBeam();
                break;
            case WALK_WAIT_FOR_END:
                if (w.origin==ORIGIN_CONNECTED) {
                    switch (cw.getPixel({x+r,y})) {
                        //case MAP_CELL_EMPTY: break;
                        case MAP_CELL_OCCUPIED:
                            bm->end = Boundary(w.p, r, cw);
                            onBoundary(w.prev_pt, IS_END, cw);
                            break;
                    }
                } else {//test this
                    if (! bm->end.tip.offline || bm->end.tip.nbp )
                        onBoundary(w.prev_pt, IS_END, cw);///
                }
                break;
            case WALK_NEW_ARC_MAYBE:
                if (w.prev_pt.with_nbp) {
                    onBoundary(w.prev_pt, IS_END, false);
                } else if (w.inclusive && w.origin==ORIGIN_CONNECTED && is_origin(beams.begin()->start)) {
                    Boundary b1 = beams.begin()->start;
                    Boundary b = Boundary(w.p, r, cw);
                    addBeam({b, w.tmp_end}); 
                }
        }
    }
    
    void Beam::onBoundary(OctPoint& op, bool is_start, CWave2& cw) {
        addCandidate(Point(op, c));
        if (op.with_nbp) {
            op.x--;
            addCandidate(Point(op, c));
            op.x++;
        }
    }
    
//CWave2
    CWave2::CWave2 (MapIf<int> &map
        #ifdef CWAVE_DEBUG
            , CWaveDebugger &debugger
        #endif
    ) :
        map_(map) 
        #ifdef CWAVE_DEBUG
            , debugger_(debugger)
        #endif        
    { }    
    
    CWave2::calc(Point src, int max_dist) {
        //if max_dist<0, then no limit
        bigBang(src, max_dist);
    }

    
    void bigBang(Point src, int max_dist) {
        stars_ = forward_list<Star>();
        candidates_to_grow_ = {};
        candidates_to_add_  = {};
        i_     = forward_list<Star>::iterator();
        p_     = forward_list<Star>::iterator();
        
        stars_.push_front(Star(src, 0));
        ins_   = stars_.begin();
        setPixel(x,y,0);
        dist_  = 2;
        
        while (!stars_.empty() && (max_dist<0 || dist_<=max_dist)) {  
            //here glx.dist is equal to the distance which is just to be explored
            //i.e., the distance of already exlored pixles is glx.dist-2
            CWAVE_DBG(display);
            starsGrow();                                  CWAVE_DBG(display);
            addCandidatesToStars();                       CWAVE_DBG(display);
            candidates_to_grow_.swap(candidates_to_add_); CWAVE_DBG(display);
            candidates_to_add_.clear();                   CWAVE_DBG(display);
        }
    }
    
    CWave2::setPixel(Point& p, int val) {
        map_.set(p.x,p.y, 0);
    }
    
    CWave2::getPixel(Point& p) {
        return map_.get(p.x,p.y);
    }
    
    void CWave2::addCandidate(const Point& c) {
        candidates_to_add_.push_back(c);
        CWAVE_DBG(highlight, c.x, c.y, PIXEL_CANDIDATE);
    }

    void CWave2::starsGrow() {
        for (i_  = stars_.begin(), p_ = stars_.before_begin(); 
                i_ != stars_.end() && i_->dist<=dist_;  
                p_  = i_, ++i_) {       CWAVE_DBG(galaxy_print);
            if (i_->empty()) {   CWAVE_DBG(dead_star, i_->x, i_->y);
                if (ins_==i_) ins_=p_;
                i_=p_;                  CWAVE_DBG(galaxy_print);
                if (i_==stars_.end() && p_!=stars_.before_begin()) 
                    break;
                stars_.erase_after(p_);
                if (stars_.empty()) 
                    return;
            } else{
                i_->grow(this);             CWAVE_DBG(display);
            }
        }                               CWAVE_DBG(galaxy_print);
        //update insertion iterator (ins_):
        for (dist_=stars_.begin()->dist, p_=ins_;
                (ins_!= stars_.end() && ins_->dist < dist_+2);
                p_=ins_, ++ins_) {      CWAVE_DBG(galaxy_print);};
        ins_=p_;                        CWAVE_DBG(galaxy_print);
    }
    
    void CWave2::addToStars() {
        bool check_diagonals=false;
        for(    std::vector<Point>::iterator cand = glx.candidates_to_grow.begin(); 
                cand != candidates_to_grow.end(); ++cand)
            if (getPixel(cand->x, cand->y) != MAP_CELL_OCCUPIED 
                && 
                (
                    getPixel(cand->x+1, cand->y+0) == MAP_CELL_EMPTY || 
                    getPixel(cand->x+0, cand->y+1) == MAP_CELL_EMPTY || 
                    getPixel(cand->x-1, cand->y+0) == MAP_CELL_EMPTY || 
                    getPixel(cand->x+0, cand->y-1) == MAP_CELL_EMPTY ||
                    (
                        check_diagonals && 
                        (
                            getPixel(cand->x+1, cand->y+1) == MAP_CELL_EMPTY || 
                            getPixel(cand->x-1, cand->y+1) == MAP_CELL_EMPTY ||
                            getPixel(cand->x-1, cand->y-1) == MAP_CELL_EMPTY || 
                            getPixel(cand->x+1, cand->y-1) == MAP_CELL_EMPTY
                        )
                    )
                )
            ) {
                ins_ = stars.insert_after(ins_, Star(c, dist));     CWAVE_DBG(display);
                ins_->grow();
            } else {
                CWAVE_DBG(highlight, cand->x, cand->y, PIXEL_REGULAR);   CWAVE_DBG(galaxy_print);
            }
        //glx.candidates_to_add.clear();
    }

    
    int markPixel(Point p, bool is_nbp) {
        #ifdef CHECK_MAP_BOUNDARIES
            if (!(p.x>=0 && p.x<map_.width() && p.y>=0 && p.y<map_.height()))
                return MAP_CELL_OUT_OF_BOUNDARY;
        #endif
        int cell = cw.getPixel(p);
        #ifdef CWAVE_DEBUG
            if (abs(sqrt(float(p.x*p.x+p.y*p.y))-0.5*(is_nbp)-s.r)>=0.5) //check the distance error
               CWAVE_DBG(highlight, p.x, p.y, PIXEL_WRONGDIST);//exit(2); //glx.map.highlight(px, py, PIXEL_WRONGDIST);
        #endif
        if (cell==MAP_CELL_EMPTY) 
            cw.setPixel(p, dist+is_nbp);
        CWAVE_DBG(display);
        return cell;
    }

    


}

    