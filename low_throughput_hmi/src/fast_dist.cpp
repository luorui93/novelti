#include <low_throughput_hmi/fast_dist.h>
#include <cmath>


namespace low_throughput_hmi_cost {


    Galaxy calculate_distances(MapIf<int> &map, int x, int y
        #ifdef PDF_STAT
            , MapIf<float> & pdf
        #endif
        #ifdef CIRC_DIV
            , vector<float>& prob_thresholds
            , MapIf<int> &divided_map
        #endif
        #ifdef TRACK_MAP
            , MapIf<int> &track_map
        #endif
        #ifdef CWAVE_DEBUG
            , CWaveDebugger &debugger
        #endif
    ) {
        return calculate_distances(map, x, y, -1
            #ifdef PDF_STAT
                , pdf
            #endif
            #ifdef CIRC_DIV
                , prob_thresholds
                , divided_map
            #endif
            #ifdef TRACK_MAP
                , track_map
            #endif
            #ifdef CWAVE_DEBUG
                , debugger
            #endif
        );
    } 
    
    Galaxy calculate_distances(MapIf<int> &map, int x, int y, int max_dist
        #ifdef PDF_STAT
            , MapIf<float> & pdf
        #endif
        #ifdef CIRC_DIV
            , vector<float>& prob_thresholds
            , MapIf<int> &divided_map
        #endif
        #ifdef TRACK_MAP
            , MapIf<int> &track_map
        #endif
        #ifdef CWAVE_DEBUG
            , CWaveDebugger &debugger
        #endif
    ) {
        Galaxy glx = galaxy_create(map, max_dist
            #ifdef PDF_STAT
                , pdf
            #endif
            #ifdef CIRC_DIV
                , prob_thresholds
                , divided_map
            #endif
            #ifdef TRACK_MAP
                , track_map
            #endif
            #ifdef CWAVE_DEBUG
                , debugger
            #endif
        );
        #ifdef CIRC_DIV
            glx.dist_thresholds.resize(prob_thresholds.size(), 0);
        #endif
        galaxy_big_bang(glx, x, y);
        return glx;
    }
    
    
    
    /** Galaxy functions  **/
    Galaxy galaxy_create(MapIf<int> &map
        #ifdef PDF_STAT
            , MapIf<float> & pdf
        #endif
        #ifdef CIRC_DIV
            , vector<float>& prob_thresholds
            , MapIf<int> &divided_map
        #endif        
            #ifdef TRACK_MAP
            , MapIf<int> &track_map
        #endif
        #ifdef CWAVE_DEBUG
            , CWaveDebugger &debugger
        #endif
    ) {
        return galaxy_create(map, -1
            #ifdef PDF_STAT
                , pdf
            #endif
            #ifdef CIRC_DIV
                , prob_thresholds
                , divided_map
            #endif
            #ifdef TRACK_MAP
                , track_map
            #endif
            #ifdef CWAVE_DEBUG
                , debugger
            #endif
        );
    }
    
    Galaxy galaxy_create(MapIf<int> &map, int max_dist
        #ifdef PDF_STAT
            , MapIf<float> & pdf
        #endif
        #ifdef CIRC_DIV
            , vector<float>& prob_thresholds
            , MapIf<int> &divided_map
        #endif
        #ifdef TRACK_MAP
            , MapIf<int> &track_map
        #endif
        #ifdef CWAVE_DEBUG
            , CWaveDebugger &debugger
        #endif
    ) {
        return Galaxy{
          //map  stars                 ins                             i                               p                               to grow  to add  dist      
            map, forward_list<Star>(), forward_list<Star>::iterator(), forward_list<Star>::iterator(), forward_list<Star>::iterator(), {},      {},     0,
            #ifdef TRACK_MAP
                //track_stars  cur_star_id  track_map
                {},          0,           track_map,
            #endif
            max_dist,
            #ifdef PDF_STAT
                0.0, pdf ,
            #endif
            #ifdef CIRC_DIV
                prob_thresholds,
                {},
                0,
                divided_map,
            #endif
            #ifdef CWAVE_DEBUG
                debugger,
            #endif
        };
        //TODO: preallocate space for candiates and track_stars
    }
    
    void galaxy_add_star_from_candidate(Galaxy& glx, const Candidate& c, int dist, bool grow){//Star star) {
        Star star = star_create_from_candidate(c, dist);
        #ifdef TRACK_MAP
            ///glx.track_map.set(star.x,star.y, c.parent_id);
            ///glx.track_stars.push_back({star.x,star.y});
            star.id = -1;///glx.cur_star_id;
            ///glx.cur_star_id++;
        #endif

        ///glx.map.set(star.x,star.y, star.dist-2);
        #ifdef MEAN_DIST
            glx.pdf_stat += glx.pdf.get(star.x,star.y)*(star.dist-2);
        #endif
        #ifdef CIRC_DIV
            
            
            glx.pdf_stat += glx.pdf.get(star.x,star.y);
        #endif
        CWAVE_DBG(highlight, star.x, star.y, PIXEL_STAR);
        glx.ins = glx.stars.insert_after(glx.ins, star);        
        if (grow)
            star_grow(glx, *(glx.ins));
    }
    
    void galaxy_add_candidate(Galaxy& glx, const Candidate& c) {
        glx.candidates_to_add.push_back(c);
        CWAVE_DBG(highlight, c.x, c.y, PIXEL_CANDIDATE);
    }       
    
    /*void galaxy_candidates_grow(Galaxy& glx) {
        for(std::vector<Candidate>::iterator cand = glx.candidates_to_grow.begin(); cand != glx.candidates_to_grow.end(); ++cand)
            if  (glx.map.get(cand->x,cand->y)==MAP_CELL_EMPTY ) {
                galaxy_add_star_from_candidate(glx, *cand, glx.dist-2);   CWAVE_DBG(galaxy_print);
            } else {
                CWAVE_DBG(highlight, cand->x, cand->y, PIXEL_REGULAR);   CWAVE_DBG(galaxy_print);
            }
    }*/

    void galaxy_stars_grow(Galaxy& glx) {
        for (glx.i = glx.stars.begin(), glx.p=glx.stars.before_begin(); glx.i!=glx.stars.end() && glx.i->dist<=glx.dist;  glx.p=glx.i, ++glx.i) {CWAVE_DBG(galaxy_print);
            //mvaddch(0,0,'0'); getch();
            if (glx.i->beams.empty()) {
                CWAVE_DBG(highlight, glx.i->x, glx.i->y, PIXEL_STARDEAD);
                if (glx.ins==glx.i) 
                    glx.ins=glx.p;
                glx.i=glx.p;                CWAVE_DBG(galaxy_print);
                if (glx.i==glx.stars.end() && glx.p!=glx.stars.before_begin()) 
                    break;
                glx.stars.erase_after(glx.p);
                if (glx.stars.empty()) 
                    return;
            } else{
                star_grow(glx, *(glx.i));   CWAVE_DBG(galaxy_print);
            }
        } CWAVE_DBG(galaxy_print);
        //update insertion iterator (glx.ins):
        for (glx.dist = glx.stars.begin()->dist, glx.p=glx.ins;
                (glx.ins!= glx.stars.end() && glx.ins->dist < glx.dist+2);
                glx.p=glx.ins, ++glx.ins) { CWAVE_DBG(galaxy_print);};
        glx.ins=glx.p;                      CWAVE_DBG(galaxy_print);
    }
    
    void add_candidates_to_stars(Galaxy& glx) {
        bool check_diagonals=false;
        for(std::vector<Candidate>::iterator cand = glx.candidates_to_grow.begin(); cand != glx.candidates_to_grow.end(); ++cand)
            if (glx.map.get(cand->x, cand->y) != MAP_CELL_OCCUPIED 
                && 
                (
                    glx.map.get(cand->x+1, cand->y+0) == MAP_CELL_EMPTY || 
                    glx.map.get(cand->x+0, cand->y+1) == MAP_CELL_EMPTY || 
                    glx.map.get(cand->x-1, cand->y+0) == MAP_CELL_EMPTY || 
                    glx.map.get(cand->x+0, cand->y-1) == MAP_CELL_EMPTY ||
                    (
                        check_diagonals && 
                        (
                            glx.map.get(cand->x+1, cand->y+1) == MAP_CELL_EMPTY || 
                            glx.map.get(cand->x-1, cand->y+1) == MAP_CELL_EMPTY ||
                            glx.map.get(cand->x-1, cand->y-1) == MAP_CELL_EMPTY || 
                            glx.map.get(cand->x+1, cand->y-1) == MAP_CELL_EMPTY
                        )
                    )
                )
            ) {
                galaxy_add_star_from_candidate(glx, *cand, glx.dist-4, true);   CWAVE_DBG(galaxy_print);
            } else {
                CWAVE_DBG(highlight, cand->x, cand->y, PIXEL_REGULAR);   CWAVE_DBG(galaxy_print);
            }
        //glx.candidates_to_add.clear();
    }
    
    void galaxy_big_bang(Galaxy& glx, int x, int y) {
        glx.map.set(x,y, 0);///
        glx.ins = glx.stars.before_begin();
        galaxy_add_star_from_candidate(glx, {x,y,if_track_then(0)},0, false);        
        glx.dist = glx.stars.begin()->dist;
        #ifdef TRACK_MAP
            glx.track_map.set(x,y,0);
        #endif
        
        
        bool late;
        while (!glx.stars.empty()) {  //mvaddch(0,0,'0'); getch();
            //here glx.dist is equal to the distance which is just to be explored
            //i.e., the distance of already exlored pixles is glx.dist-2
            CWAVE_DBG(galaxy_print);
            //add_candidates_to_stars(glx);
            if ( glx.max_dist>0 && glx.dist>=glx.max_dist )
                break;
            galaxy_stars_grow(glx);         CWAVE_DBG(galaxy_print);
            
            //galaxy_candidates_grow(glx);
            add_candidates_to_stars(glx);   CWAVE_DBG(galaxy_print);

            glx.candidates_to_grow.swap(glx.candidates_to_add);  CWAVE_DBG(galaxy_print);
            glx.candidates_to_add.clear();  CWAVE_DBG(galaxy_print);
        }
    }
    

    
    
    
    /** Star functions **/
    
    Star star_create(int x, int y, int init_dist) {
        return star_create_from_candidate({x, y, if_track_then(-1)}, init_dist);
    }
    
    Star star_create_from_candidate(const Candidate& c, int init_dist) {
        forward_list<Beam> beams; //octant dx dy x  y  leps eps tip 
        Beam full_circle_beam = {  {0,     2, 0, 1, 0, -1,  0,  {0}}, 
                                   {7,     2, 0, 1, 0, -1,  0,  {0}} };
        beams.push_front(full_circle_beam);
                     //state              inclusive origin           curr           prev                  prev_pt     tmp_end;
        StarWalk walk={WALK_JUST_STARTED, false,    ORIGIN_AT_START, beams.begin(), beams.before_begin(), {0},        {0}};
        //          x,    y,  r, dist         beams  walk         
        return Star{c.x, c.y, 1, init_dist+2, beams, walk};
    }

    void star_set_limited_range(Star& s, int sx, int sy, int ex, int ey) {
        s.beams.begin()->start = boundary_create(sx, sy, 0);
        s.beams.begin()->end   = boundary_create(ex, ey, 0);
    }
    
    void star_add_beam(Star& s, Beam b) {
        s.walk.prev = s.walk.curr;
        s.walk.curr = s.beams.insert_after(s.walk.curr, b);
    }
    
    void star_delete_current_beam(Star &s) {
        s.walk.curr=s.walk.prev;
        s.beams.erase_after(s.walk.prev); 
    }
    
    #define is_origin(boundary) ((boundary).y==0 && (boundary).octant==0)
    void star_grow(Galaxy& glx, Star& s) {
        CWAVE_DBG(highlight, s.x,s.y, PIXEL_STAR);
        s.walk.origin = (!s.beams.empty() && is_origin(s.beams.begin()->start)) ? ORIGIN_AT_START : ORIGIN_NOT_AT_START;
        for (s.walk.curr = s.beams.begin(), s.walk.prev = s.beams.before_begin(); s.walk.curr != s.beams.end(); s.walk.prev=s.walk.curr, ++s.walk.curr) {
            beam_grow(glx, s, *(s.walk.curr));
        }
        s.r++;
        s.dist+=2;
    }
   


    
    
    /** Boundary functions **/
    Boundary boundary_create(int dx, int dy, int eps) {
        int x, y;
        char oct; //octant
        if (abs(dx)>=abs(dy))
            if (dx>0)
                if (dy>=0)  { x=+dx; y=+dy; oct=0+(dx==dy);}
                else        { x=+dx; y=-dy; oct=7;}
            else
                if (dy>0)   { x=-dx; y=+dy; oct=3;}
                else        { x=-dx; y=-dy; oct=4+(dx==dy);}
        else
            if (dy>=0)
                if (dx>0)   { x=+dy; y=+dx; oct=1;}
                else        { x=+dy; y=-dx; oct=2;}
            else
                if (dx>=0)  { x=-dy; y=+dx; oct=6;}
                else        { x=-dy; y=-dx; oct=5;}
        return     //octant dx dy x  y  leps   eps  tip 
            Boundary{oct,   x, y, 0, 0, y-x/2, eps, {0}};
    }
    
    Candidate candidate_create_from_octpoint(Galaxy& glx, Star& star, OctPoint p) {
        switch (p.oct) {
            case 0: return { star.x+p.x, star.y+p.y, if_track_then(star.id) };
            case 1: return { star.x+p.y, star.y+p.x, if_track_then(star.id) };
            case 2: return { star.x-p.y, star.y+p.x, if_track_then(star.id) };
            case 3: return { star.x-p.x, star.y+p.y, if_track_then(star.id) };
            case 4: return { star.x-p.x, star.y-p.y, if_track_then(star.id) };
            case 5: return { star.x-p.y, star.y-p.x, if_track_then(star.id) };
            case 6: return { star.x+p.y, star.y-p.x, if_track_then(star.id) };
            case 7: return { star.x+p.x, star.y-p.y, if_track_then(star.id) };
        }
    }
    
    bool boundary_add_pixel(Galaxy& glx, Star& star, Boundary& line) { //returns true if y is incremented
        bool incremented=false;
        line.x++;
        if (line.leps>0) {
            line.leps -= line.dx;
            line.y++;
            incremented = true;
        }
        line.leps += line.dy;
        CWAVE_DBG(highlight_boundary, star, line);
        ///galaxy_add_candidate(glx, candidate_create_from_octpoint(glx,star,{line.octant, line.x, line.y, 0, false}));
        return incremented;
    }

    void boundary_calculate_arc_tip(Galaxy& glx, Star& star, Boundary& b, int radius) {
        b.tip = {b.x, b.y, b.eps, false, false};
        //eps = x^2+y^2-r^2
        if (b.eps-b.x>=0) {    //Start pixel is off the boundary line
            b.tip.offline = true;
        } else {                            //Start pixel is on the boundary line
            b.eps += 2*b.x+1;               // eps -> eps(x+1,y,r)
            if (boundary_add_pixel(glx, star, b)) {
                b.eps += 2*(b.y-radius-1); // eps -> eps(x+1,y+1,r+1)
                return;
            }
        }

        //Check condition for non-Bresenham point (NBP)
        b.eps += 2*(b.x-radius);           //eps -> eps(x+1,y,r+1) 
        if (b.eps-b.x<=0) {   //Non-Bresenham point
            b.tip.nbp = true;
            if (boundary_add_pixel(glx, star, b))
                b.eps += 2*b.y-1;           // eps -> eps(x+1,y+1,r+1)
        } else {                            //Regular (Bresenham) point
            b.eps -= 2*b.x+1;               // eps -> eps(x,y,r+1)
        }
    }
    

    
    Boundary boundary_create_and_grow(Galaxy& glx, Star& star, OctPoint& pt, int radius) {
        ///galaxy_add_candidate(glx, candidate_create_from_octpoint(glx,star,pt));
        int x = pt.x;
        int eps = pt.F-1 + (pt.oct%2 ?  2*pt.y-x : x-2*pt.y);
        Boundary b = {pt.oct, x, pt.y, x, pt.y, pt.y-x/2, eps, {0}};
        boundary_calculate_arc_tip(glx, star, b, radius);
        return b;
    }
    
    /** Beam functions **/    
    void beam_grow(Galaxy& glx, Star& star, /**/Beam& b) {
        star.walk.state = WALK_JUST_STARTED;
        star.walk.inclusive = false;
        
        //calculate arc start and end tips
        boundary_calculate_arc_tip(glx, star, b.start, star.r);
        boundary_calculate_arc_tip(glx, star, b.end, star.r);
        beam_arc_walk(glx, star, b, b.start, b.end);
            
        switch (star.walk.state) {
            case WALK_JUST_STARTED: 
            case WALK_WAIT_FOR_START:
                if (star.walk.inclusive)
                     pixel_on_boundary_process(glx, star,  star.walk.prev_pt, IS_END, true);
                star_delete_current_beam(star);
                break;
            case WALK_WAIT_FOR_END:
                if (star.walk.origin==ORIGIN_CONNECTED) {
                    switch (glx.map.get(star.x+star.r,star.y)) {
                        //case MAP_CELL_EMPTY: break;
                        case MAP_CELL_OCCUPIED:
                            star.walk.curr->end = boundary_create_and_grow(glx, star, star.walk.prev_pt, star.r);
                            pixel_on_boundary_process(glx, star,  star.walk.prev_pt, IS_END, false);
                            break;
                    }
                } else {//test this
                    if (! star.walk.curr->end.tip.offline || star.walk.curr->end.tip.nbp )
                        pixel_on_boundary_process(glx, star,  star.walk.prev_pt, IS_END, false);
                }
                break;
            case WALK_NEW_ARC_MAYBE:
                if (star.walk.prev_pt.with_nbp) {
                    pixel_on_boundary_process(glx, star,  star.walk.prev_pt, IS_END, false);
                } else if (star.walk.inclusive && star.walk.origin==ORIGIN_CONNECTED && is_origin(star.beams.begin()->start)) {
                    Boundary b1 = star.beams.begin()->start;
                    Boundary b = boundary_create_and_grow(glx, star, star.walk.prev_pt, star.r);
                    star_add_beam(star, {b, star.walk.tmp_end}); 
                }
        }
    }
    
    #define octant_even_loop(oct, exit_condition, condition2)\
        while ( exit_condition ) {\
            /*should be: F = x*x+y*y-star.r*star.r-x+2*y+1;*/\
            if (pixel_pair_visit(glx, star, beam, oct, x, y, F, true, nbp))\
                return;\
            nbp = (F>=0) && ((F-=2*(--x)) + 2*(2*x-star.r)+1 < 0);\
            F += 2*(++y)+1;\
        }\
        if (pixel_pair_visit(glx, star, beam, oct, x, y, F, (condition2), nbp))\
            return;

    #define octant_odd_loop(oct, exit_condition) \
        while ( exit_condition ) {\
            x_ = x; F_ = F; /*should be: F = x*x+y*y-star.r*star.r+x-2*y+1;*/\
            nbp = (F<0) && ( (F+=2*(++x)) + 2*(y-star.r-1) < 0 );\
            if (pixel_pair_visit(glx, star, beam, oct, x_, y, F_, true, nbp))\
                return;\
            F += -2*(--y)+1; \
        }

    #define octant_even(oct)\
        if (end_octant==(oct)) {\
            octant_even_loop( oct, y<end_tip_y, x==end_tip_x );\
            return;\
        } \
        octant_even_loop( oct, y<x, false );\
        if (x!=y)\
            {x++;y--;};\
        F -= (x+y);

    #define octant_odd(oct)\
        nbp=false;\
        if (end_octant==(oct)) {\
            octant_odd_loop( oct, y>end_tip_y );\
            if (y>=end_tip_y) \
                if (pixel_pair_visit(glx, star, beam, (oct), x, y, F, (y>=end_tip_y), end_tip_nbp))\
                    return;\
            return;\
        }\
        octant_odd_loop( oct, y>0 );\
        x=star.r; y=0; F =1-x; 


    void beam_arc_walk(Galaxy& glx, Star& star, Beam& beam, /**/ const Boundary& start, const Boundary& end) {
        int x_, F_, x=start.tip.x, y=start.tip.y, F=start.tip.eps+1, 
            end_tip_x=end.tip.x, end_tip_y=end.tip.y, end_octant=end.octant;
        bool nbp = start.tip.nbp, end_tip_nbp=end.tip.nbp;        
        if (start.octant%2==0) {
            F -= x-2*y;
            if (start.tip.offline) 
                F-= 2*(--x);
        } else {
            F += x-2*y;
            if (start.tip.offline) {
                if (pixel_pair_visit(glx, star, beam, start.octant, x-1, y, F-2*x, false, start.tip.nbp))
                    return;
                F-= 2*(--y)-1; 
            }
        }

        switch(start.octant) {
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

    
    
    /* Pixel functions */
    bool pixel_pair_visit(Galaxy& glx, Star& star, Beam& beam, /**/int oct, int x, int y, int F, bool reg, bool nbp) {
        //returns true if beam_arc_walk should return
        int cost=MAP_CELL_EMPTY;
        bool with_nbp=false;
        if (reg) {
            if (oct==7 && y==0 && star.walk.origin==ORIGIN_AT_START) {
                star.walk.origin=ORIGIN_CONNECTED; 
                return false;
            }
            cost = pixel_check_and_mark(glx, star, oct, x, y, false);
        } else if (!nbp)
            return false;
        if (cost!=MAP_CELL_OCCUPIED && nbp) { 
            //process_beam_pixel(glx, star, beam, oct, x, y, F, cost);
            F+=2*((x++) +(oct%2));
            cost = pixel_check_and_mark(glx, star, oct, x, y, true);
            with_nbp=reg;
        }
        return pixel_in_beam_process(glx, star, beam, oct, x, y, F, cost, with_nbp);
    }
    
    
    
    int pixel_check_and_mark(Galaxy& glx, Star& star, char oct, int x, int y, bool is_nbp) {
        int px, py;  //absolute coordinates
        switch (oct) {
                case 0: px=star.x+x; py=star.y+y; break;
                case 1: px=star.x+y; py=star.y+x; break;
                case 2: px=star.x-y; py=star.y+x; break;
                case 3: px=star.x-x; py=star.y+y; break;
                case 4: px=star.x-x; py=star.y-y; break;
                case 5: px=star.x-y; py=star.y-x; break;
                case 6: px=star.x+y; py=star.y-x; break;
                case 7: px=star.x+x; py=star.y-y; break;
        }
        #ifdef CHECK_MAP_BOUNDARIES
            if (!(px>=0 && px<glx.map.width() && py>=0 && py<glx.map.height()))
                return MAP_CELL_OUT_OF_BOUNDARY;
        #endif
        int cell = glx.map.get(px,py);
        #ifdef COLORED11
            if (cell>0) //check if the point has been previously visited
                CWAVE_DBG(highlight, px, py, PIXEL_OVERLAP); //exit(2);
        #endif
        #ifdef CWAVE_DEBUG
            if (abs(sqrt(float(x*x+y*y))-0.5*(is_nbp)-star.r)>=0.5) //check the distance error
               CWAVE_DBG(highlight, px, py, PIXEL_WRONGDIST);//exit(2); //glx.map.highlight(px, py, PIXEL_WRONGDIST);
        #endif
        if (cell==MAP_CELL_EMPTY) {
            glx.map.set(px,py, star.dist+is_nbp);
            #ifdef MEAN_DIST
                glx.pdf_stat += glx.pdf.get(px,py)*(star.dist+is_nbp);
            #endif
            #ifdef CIRC_DIV
                float p = glx.pdf.get(px,py);
                if (glx.pdf_stat + p > glx.prob_thresholds[glx.cur_region] && glx.cur_region<glx.prob_thresholds.size()-1) {
                    glx.cur_region++;
                    glx.pdf_stat=0.0;
                }
                glx.pdf_stat += p;
                glx.divided_map.set(px,py, glx.cur_region);
                //printf("START pdf_stat=%f, cur_region=%d, prob_thresholds[cur_region]=%f\n", glx.pdf_stat,glx.cur_region, glx.prob_thresholds[glx.cur_region]);
                //printf("END pdf_stat=%f, cur_region=%d, prob_thresholds[cur_region]=%f\n", glx.pdf_stat,glx.cur_region, glx.prob_thresholds[glx.cur_region]);
            #endif
            #ifdef TRACK_MAP
                if (star.id==-1) {
                    star.id = glx.cur_star_id++;
                    glx.track_stars.push_back({star.x,star.y});
                }
                glx.track_map.set(px,py, star.id);
            #endif
        }
        CWAVE_DBG(map);
        return cell;
    }

    
    bool pixel_in_beam_process(Galaxy& glx, Star& star, Beam& beam, /**/char oct, int x, int y, int F, int cost, bool with_nbp) {
        // returns true if beam_arc_walk should finish
        int action = 0;
        bool inclusive = (star.walk.state== WALK_JUST_STARTED ? (cost>=0): star.walk.inclusive);
        star.walk.inclusive = (cost>=0);
        if ((cost==MAP_CELL_EMPTY && !inclusive) || cost>=0)
            star.walk.prev_pt = {oct, x, y, F, with_nbp};
        if (cost==MAP_CELL_EMPTY) {
            switch(star.walk.state) {
                case WALK_JUST_STARTED:
                    if (! star.walk.curr->start.tip.offline || star.walk.curr->start.tip.nbp)                        
                        pixel_on_boundary_process(glx, star, star.walk.prev_pt, IS_START, inclusive);
                    break;
                case WALK_WAIT_FOR_START: 
                    action=1;
                    break;
                case WALK_NEW_ARC_MAYBE: 
                    action=2;
                    break;
            }
            star.walk.state =  WALK_WAIT_FOR_END;
        } else { //occupied or previously visited pixel
            switch(star.walk.state) {
                case WALK_JUST_STARTED:     star.walk.state =  WALK_WAIT_FOR_START;
                    if (with_nbp)
                        pixel_on_boundary_process(glx, star, {oct, x, y, F, with_nbp}, IS_START, inclusive);
                    break;
                case WALK_WAIT_FOR_START:
                    if (inclusive & cost==MAP_CELL_OCCUPIED)        //test 13 => test 15
                        pixel_on_boundary_process(glx, star,  star.walk.prev_pt, IS_END, inclusive);      //test 13
                    break;                
                case WALK_WAIT_FOR_END:     star.walk.state =  WALK_NEW_ARC_MAYBE;
                    action=3;
                    break;
            }
        }
        if (action>0) {
            Boundary b = boundary_create_and_grow(glx, star, star.walk.prev_pt, star.r);
            if      (action==1) {
                star.walk.curr->start = b; 
            } else if (action==2){
                star_add_beam(star, {b, star.walk.tmp_end}); 
            } else {
                star.walk.tmp_end = star.walk.curr->end; //TODO is this a copy?                    
                star.walk.curr->end = b; 
            }
            pixel_on_boundary_process(glx, star,  star.walk.prev_pt, action!=3, inclusive); 
        }
        
        if ((cost>=-1 && inclusive) || (cost==MAP_CELL_OCCUPIED && with_nbp))
            star.walk.prev_pt = {oct, x, y, F, with_nbp};
        CWAVE_DBG(star_print, star);
        CWAVE_DBG(map);

        return false;
    }

    
    void pixel_on_boundary_process(Galaxy& glx, Star& star, OctPoint p, bool is_start, bool inclusive) {
        if (1 || !inclusive) {
            galaxy_add_candidate(glx, candidate_create_from_octpoint(glx,star,p));
            p.x--;
            if (p.with_nbp)
                galaxy_add_candidate(glx, candidate_create_from_octpoint(glx,star,p));
        }
        /*
        int px = star.x, x = p.x, 
            py = star.y, y = p.y,
            dx = 0, dy = 0, vx=0, vy=0;
        
        Boundary* b = is_start ? &(star.walk.curr->start) : &(star.walk.curr->end);
        bool boundary_incremented = b->y != y;
        char oct = p.oct;
        //y += (is_start xor oct%2)  ? -1 : 1; //step aside
        switch (oct) {
            case 0: px+=x; py+=y; dx--; vy++; break;
            case 1: px+=y; py+=x; dy--; vx++; break;
            case 2: px-=y; py+=x; dy--; vx--; break;
            case 3: px-=x; py+=y; dx++; vy++; break;
            case 4: px-=x; py-=y; dx++; vy--; break;
            case 5: px-=y; py-=x; dy++; vx--; break;
            case 6: px+=y; py-=x; dy++; vx++; break;
            case 7: px+=x; py-=y; dx--; vy--; break;
        }
        //CWAVE_DBG(highlight, px, py, PIXEL_EDGE);
        
        if ((is_start xor oct%2)  &&  boundary_incremented  && (!p.with_nbp || glx.map.get(px,py)!=MAP_CELL_OCCUPIED) &&  glx.map.get(px-dx,py-dy)==MAP_CELL_EMPTY )
            galaxy_add_candidate(glx, {px-dx, py-dy,    if_track_then(star.id)});
        if (is_start xor oct%2) {
            px-=vx; py-=vy;
        } else {
            px+=vx; py+=vy;
        }
        
        
        if (p.with_nbp) {
            px+=dx; py+=dy;
            if (b->tip.nbp && glx.map.get(px,py)!=MAP_CELL_EMPTY && glx.map.get(px-dx,py-dy)==MAP_CELL_EMPTY) {
                    px-=dx; py-=dy;
            } 
        }  
        
        if (glx.map.get(px,py)==MAP_CELL_EMPTY)
            galaxy_add_candidate(glx, {px,py,    if_track_then(star.id)});
        /**/
    }
        
}

    