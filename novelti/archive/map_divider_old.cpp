#define TRACK_MAP 1
#define CIRC_DIV 1
#include "fast_dist.cpp"

#include <math.h>
#include <cmath>

namespace novelti {

#ifdef DEBUG_DIVIDER
class MapDividerDebugger {
    public:
        std::ostringstream msg_stream_;
        virtual ~MapDividerDebugger() {};
        virtual void text() {};
        virtual void pose(int x, int y)=0;
        virtual void border_pose(int x, int y, char oct)=0;
        virtual void dist_map()=0;
        virtual void track_map(MapIf<int>& track_map)=0;
        virtual void stars(vector<Point2D>& stars)=0;
};
#endif

class MapDivider {

    public:
        #ifdef DEBUG_DIVIDER
            MapDividerDebugger* debugger_;
            #define DEBUG_MSG(my_stream) debugger_->msg_stream_ << my_stream; debugger_->text(); debugger_->msg_stream_.str(""); debugger_->msg_stream_.clear();
            #define DEBUG_IT(type, ...) debugger_->type( __VA_ARGS__ );
        #else
            #define DEBUG_MSG(...)
            #define DEBUG_IT(type, ...)
        #endif
        
        enum DivisionPolicy {
            VERT_TILES,
            HORIZ_TILES,
            BY_EXTREMALS,
            BY_EQUIDISTS,
            MIXED_METHOD1,
            MIXED_METHOD2,
            WRONG
        };
            
        struct BorderWalker {
            Point2D brd;
            Point2D cnt;
            char oct;            
            int bstar;
        };
        
        static const int NEIGHBOURS[8][2];
        static const int DIFF[2][8][2];
        
        MapIf<int>& map_;
        MapIf<int>& divided_map_;
        
        MapIf<float>* pdf_;
        vector<float> optimal_probs_;
        vector<float> actual_probs_;
        float cur_prob_;
        char cur_region_;
        int count_;
        
        //specific to extermal divider:
        //border walker:
        ///OctPoint w_;
        BorderWalker w_;
        /*int wx_;
        int wy_;
        int woct_;
        int wstar_;*/
        
        MapDivider(MapIf<int>& map, MapIf<int>& divided_map) :
            map_(map), 
            divided_map_(divided_map) 
        { count_ =0; }

        void processPixel(int x, int y) {
            int reg = divided_map_.get(x,y);
            //DEBUG_IT(pose, x,y);
            //DEBUG_MSG("pixel ("<<x<<","<<y<<"), region=" << reg << "set to " <<cur_region_);
            if (reg==-1) {
                //sd.map.highlight(x,y, (sd.cur_region%5)+PIXEL_REGION); sd.map.display();
                float p = pdf_->get(x,y);
                if (    cur_prob_+p   > optimal_probs_[cur_region_] && 
                        cur_region_ != optimal_probs_.size()-1) {
                    DEBUG_MSG("     incrementing region at (" << x << "," << y << ")" ); 
                    actual_probs_[cur_region_] = cur_prob_;
                    cur_region_++;
                    cur_prob_ = 0.0;
                }
                divided_map_.set(x,y, cur_region_);
                cur_prob_ += p;
                ///DEBUG_MSG("space_divider: (%d,%d), cur_prob=%f, cur_region=%d", x,y, cur_prob_, cur_region_);
            }
        }
        

        #define LINE_WALK_MACRO(xx, yy, func, ...) \
            do {\
                if (leps>0) {leps -= dx; y++;}\
                leps += dy; x++;\
                func(start.x+(xx), start.y+(yy), ##__VA_ARGS__);\
            } while (x<dx); \
            break;
    
        void segmentWalk(Point2D start, Point2D end) {
            int dx = end.x-start.x, 
                dy = end.y-start.y,
                x, y;
            char oct;
            
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
        
            dx=x; dy=y; x=0; y=0;
            int leps = dy-dx/2;
            
            processPixel(start.x, start.y);
            switch(oct) {
                case 0: LINE_WALK_MACRO( x,  y, processPixel); 
                case 1: LINE_WALK_MACRO( y,  x, processPixel); 
                case 2: LINE_WALK_MACRO(-y,  x, processPixel); 
                case 3: LINE_WALK_MACRO(-x,  y, processPixel); 
                case 4: LINE_WALK_MACRO(-x, -y, processPixel); 
                case 5: LINE_WALK_MACRO(-y, -x, processPixel); 
                case 6: LINE_WALK_MACRO( y, -x, processPixel); 
                case 7: LINE_WALK_MACRO( x, -y, processPixel); 
            }/**/
        }
        
        void repaintPixel(int x, int y, MapIf<int>& src_map, MapIf<int>& dest_map, int old_val, int new_val) {
            if (src_map.get(x,y) == old_val) 
                dest_map.set(x,y, new_val);
        }
        
        void repaint_segment(Point2D start, Point2D end, MapIf<int>& track_map, MapIf<int>& dest_map, int old_star, int new_star) {
            int dx = end.x-start.x, 
                dy = end.y-start.y,
                x, y;
            char oct;
            
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
        
            dx=x; dy=y; x=0; y=0;
            int leps = dy-dx/2;
            
            repaintPixel(start.x, start.y, track_map, dest_map, old_star, new_star);
            switch(oct) {
                case 0: LINE_WALK_MACRO( x,  y, repaintPixel, track_map, dest_map, old_star, new_star); 
                case 1: LINE_WALK_MACRO( y,  x, repaintPixel, track_map, dest_map, old_star, new_star); 
                case 2: LINE_WALK_MACRO(-y,  x, repaintPixel, track_map, dest_map, old_star, new_star); 
                case 3: LINE_WALK_MACRO(-x,  y, repaintPixel, track_map, dest_map, old_star, new_star); 
                case 4: LINE_WALK_MACRO(-x, -y, repaintPixel, track_map, dest_map, old_star, new_star); 
                case 5: LINE_WALK_MACRO(-y, -x, repaintPixel, track_map, dest_map, old_star, new_star); 
                case 6: LINE_WALK_MACRO( y, -x, repaintPixel, track_map, dest_map, old_star, new_star); 
                case 7: LINE_WALK_MACRO( x, -y, repaintPixel, track_map, dest_map, old_star, new_star); 
            }/**/
        }
        
        void pathWalk(MapIf<int>& track_map, vector<Point2D>& track_stars, int x, int y) {
            DEBUG_MSG("border px: ("<<x<<","<<y<<")");
            //DEBUG_IT(border_pose, x,y, 0);
            int parent_id = track_map.get(x,y);
            Point2D start = {x,y};
            Point2D end = track_stars[parent_id];
            segmentWalk(start, end);
            /*do {
                star_id = track_map[x][y];
                Point2D end = stars[star_id];
                segmentWalk(start, end);
            } while (star_id!=0);*/
        }
    
        Point2D findBorderPoint(Point2D& p) {
            int x=p.x, y=p.y;
            for (; map_.get(x,y) != MAP_CELL_OCCUPIED; x++)
                processPixel(x, y);
            //DEBUG_MSG("space_divider: found border point = (%d,%d)", p.x,p.y);
            return {x,p.y};
        }

#define oct_update_a(oct)  if (WALK_STYLE==0) {(oct)-=1; (oct)&=7;}\
                           else               {(oct)-=1+((oct)%2==0); (oct)&=7;}
#define oct_update_b(oct)  if (WALK_STYLE==0) {(oct)+=2; (oct)&=6;}\
                           else               {(oct)+=1; (oct)&=7;}


        #define WALK_STYLE 0
        #define COUNT_LIM 3
        void walkPetal(MapIf<int>& track_map, vector<Point2D>& track_stars, int star, bool early_stop) {
            DEBUG_MSG("start petal w.brd={"<<w_.brd.x<<","<<w_.brd.y<<"}, oct="<<int(w_.oct)<<", w.bstar="<<w_.bstar<<", star="<<star);
            Point2D start     = w_.brd;
            char    start_oct = w_.oct;
            Point2D tmp;
            int tstar;
            do {
                //if (count_ == COUNT_LIM)
                //QQQ//DEBUG_IT(border_pose, w_.brd.x, w_.brd.y, w_.oct);
                tmp = {w_.brd.x + DIFF[WALK_STYLE][w_.oct][0],  w_.brd.y + DIFF[WALK_STYLE][w_.oct][1]};
                tstar = track_map.get(tmp.x,tmp.y);    
                if (tstar==star) { //if {tx,ty}'s source star == current petal star
                    //that means t is an inner pixel
                    oct_update_a(w_.oct);
                    w_.cnt = tmp;   //move inner pixel
                    pathWalk(track_map, track_stars, tmp.x, tmp.y); 
                } else {
                    //means t is not an inner pixel
                    oct_update_b(w_.oct);
                    w_.brd = tmp; //move border pixel, inner pixel stays same
                    w_.bstar = tstar;
                }
                //if (count_ == COUNT_LIM)
                //QQQ//DEBUG_IT(border_pose, w_.brd.x, w_.brd.y, w_.oct);
                if (w_.brd.x==start.x && w_.brd.y==start.y && (early_stop || w_.oct==start_oct))
                    break;
                if (w_.bstar>0 && //means we stand on another petal
                        track_stars[w_.bstar].x == w_.cnt.x && 
                        track_stars[w_.bstar].y == w_.cnt.y &&  w_.oct%2==0) { // we found a new star
                    //flip walker
                    tmp = w_.brd; w_.brd = w_.cnt; w_.cnt = tmp;
                    w_.oct+=4; w_.oct&=7; 
                    tstar = w_.bstar;
                    w_.bstar = star;
                    walkPetal(track_map, track_stars, tstar, false);
                    //flip back;
                    w_.bstar = tstar;
                    tmp = w_.brd; w_.brd = w_.cnt; w_.cnt = tmp;
                    w_.oct+=4; w_.oct&=7; 
                }

            } while ( 1) ;//!(w_.brd.x==start.x && w_.brd.y==start.y && w_.oct==start_oct) );
            DEBUG_MSG("end petal w.brd={"<<w_.brd.x<<","<<w_.brd.y<<"}, oct="<<int(w_.oct)<<", w.bstar="<<w_.bstar<<", star="<<star);
            //cout<< "end petal w.brd={"<<w_.brd.x<<","<<w_.brd.y<<"}, oct="<<w_.oct<<", w.bstar="<<w_.bstar<<", star="<<star<<endl;
        }
        
        /*void borderStep(MapIf<int>& track_map, vector<Point2D>& track_stars, 
                        int dx, int dy, char oct_occ, char oct_free) { //__attribute__((always_inline));
            int d, y, x, star;
            x = wx_+dx; //wx_ and wy_ stay on the obstacle
            y = wy_+dy;
            star = track_map.get(x,y);
            if (star!=wstar_) {
                if (star<0 || //star<0 means it's an obstacle
                    (track_map.get(track_stars[star].x,   track_stars[star].y  ) != wstar_ && 
                     track_map.get(track_stars[wstar_].x, track_stars[wstar_].y) != star)
                ) {
                    wx_ = x; 
                    wy_ = y; 
                    woct_ = oct_occ; 
                    return;
                }
            } 
            wstar_ = star; 
            woct_  = oct_free; 
            pathWalk(track_map, track_stars, x, y); 
            return;
        }* /
        
        void walkBorder(MapIf<int>& track_map, vector<Point2D>& track_stars, Point2D& start) {

        }*/
        
        void divide_by_equidists(Point2D& center, MapIf<int>& track_map) {
            DEBUG_MSG("dividing by EQUIDISTS");
            Galaxy glx = calculate_distances(map_, center.x, center.y, *pdf_, optimal_probs_, divided_map_, track_map);
            //DEBUG_IT(dist_map);
            DEBUG_IT(track_map, track_map);
            DEBUG_IT(stars, glx.track_stars);
            
            /*DEBUG_MSG("glx.dist_thresholds={"<<glx.dist_thresholds[0]<<","<<glx.dist_thresholds[1]<<","<<glx.dist_thresholds[2]<<","<<glx.dist_thresholds[3]<<"}");
            actual_probs_ = vector<float>(optimal_probs_.size(), 0.0);
            int region;
            for(int x=0; x<divided_map_.width(); x++)
                for (int y=0;y<divided_map_.height(); y++) {
                    int dist = map_.get(x,y);
                    if (dist>=0) {
                        //DEBUG_MSG("map_.get("<<x<<","<<y<<")="<<dist);
                        for (region=0; glx.dist_thresholds[region]<dist && region<glx.dist_thresholds.size()-1 ; region++);
                        actual_probs_[region] += pdf_->get(x,y);
                        //DEBUG_MSG("region" << region);
                        divided_map_.set(x,y,region);
                    }
                };*/
            map_.clean_dist();
        }
        
        #define WALK_STYLE 0
        void repaint_region(MapIf<int>& track_map, MapIf<int>& dest_map, Point2D& border_pt, Point2D& star_pt, int old_val, int new_val, bool same_map) {
            DEBUG_MSG("        started repainting region border pt at ("<<border_pt.x<<","<<border_pt.y<<") from "<<old_val<<" to "<<new_val<<" ");
            Point2D brd = {border_pt.x,border_pt.y};
            Point2D cnt = {border_pt.x-1,border_pt.y};
            char    oct = 4;
            Point2D start     = brd;
            char    start_oct = oct;
            Point2D tmp;
            repaint_segment(cnt, star_pt, track_map, dest_map, old_val, new_val);
            do {
                //QQQ//DEBUG_IT(border_pose, brd.x, brd.y, oct);
                tmp = {brd.x + DIFF[WALK_STYLE][oct][0],  brd.y + DIFF[WALK_STYLE][oct][1]};
                
                if (track_map.get(tmp.x,tmp.y)==old_val || (same_map && track_map.get(tmp.x,tmp.y)==new_val)) { //it's an inner pixel
                    oct_update_a(oct);
                    cnt = tmp;                             //move inner pixel
                    //if (oct%2==0)
                        repaint_segment(cnt, star_pt, track_map, dest_map, old_val, new_val);                    

                } else {                        //it's a border pixel
                    oct_update_b(oct);
                    brd = tmp;                  //move border pixel, inner pixel stays same
                }
                //QQQ//DEBUG_IT(border_pose, brd.x, brd.y, oct);
            } while ( !(brd.x==start.x && brd.y==start.y && oct==start_oct) );
            DEBUG_MSG("        finished repainting region border pt at ("<<border_pt.x<<","<<border_pt.y<<") from "<<old_val<<" to "<<new_val<<" ");
            
        }
        
        #define WALK_STYLE 0
        void investigate_region(MapIf<int>& track_map, vector<Point2D>& track_stars, Point2D& center, int px, int py) {
            int star_id  = track_map.get(px,py);
            bool vis_x = false, vis_y = false;
            Point2D star = track_stars[star_id];            
            DEBUG_MSG("investiagting region starting from ("<<px<<","<<py<<"), current star="<<star_id<<", located at ("<<star.x<<","<<star.y<<")");
            
            //find border point
            int x=px+1, y=py;
            for (int v = track_map.get(px,py);
                 track_map.get(x,y) == v; 
                    x++);
            DEBUG_MSG("        found border point = ("<<x<<","<<y<<")");
            
            
            //walk around the border
            DEBUG_MSG("        starting walking around border");
            bool separated_region = true;
            Point2D brd = {x,y};
            Point2D cnt = {x-1,y};
            char    oct = 4;
            Point2D start     = brd;
            char    start_oct = oct;
            Point2D tmp;
            float dist, min_dist = std::numeric_limits<float>::max();
            Point2D new_star = {0};
            do {
                //QQQ//DEBUG_IT(border_pose, brd.x, brd.y, oct);
                if (brd.x==star.x && brd.y==star.y) { //not a separated region
                    separated_region = false;
                    break;
                }
                if (separated_region  &&  oct%2==0 && track_map.get(brd.x,brd.y) > 0) {
                    dist = sqrt((brd.x-star.x)*(brd.x-star.x) + (brd.y-star.y)*(brd.y-star.y));
                    if (dist < min_dist) {
                        min_dist = dist;
                        new_star = brd;
                        DEBUG_MSG("    --- found potential new star at ("<<brd.x<<","<<brd.y<<")");
                    }
                }
                tmp = {brd.x + DIFF[WALK_STYLE][oct][0],  brd.y + DIFF[WALK_STYLE][oct][1]};
                if (track_map.get(tmp.x,tmp.y)==star_id) { //it's an inner pixel
                    oct_update_a(oct);
                    cnt = tmp;                             //move inner pixel
                    if (cnt.x==center.x) vis_x=true;
                    if (cnt.y==center.y) vis_y=true;                    
                } else {                        //it's a border pixel
                    oct_update_b(oct);
                    brd = tmp;                  //move border pixel, inner pixel stays same
                }
                //QQQ//DEBUG_IT(border_pose, brd.x, brd.y, oct);
            } while ( !(brd.x==start.x && brd.y==start.y && oct==start_oct) );
            DEBUG_MSG("    finished border walk. separated_retion=="<<separated_region<<", new_star=("<<new_star.x<<","<<new_star.y<<")");
            
            
            
            //if separated region, add star, repaint region on track_map
            if (separated_region && (star_id!=0 || (vis_x==false || vis_y==false)) ) {
                star = new_star;
                track_stars.push_back({star.x,star.y});
                DEBUG_MSG("    >>> new star added at ("<<star.x<<","<<star.y<<")");
                repaint_region(track_map, track_map, start, star, star_id, track_stars.size()-1, true);
                star_id = track_stars.size()-1;
            }
            
            //fill region on map
            repaint_region(track_map, map_, start, star, star_id, 1, false);
            DEBUG_MSG("Finished region investigation");
        }
        
        void fix_individual_regions(MapIf<int>& track_map, vector<Point2D>& track_stars, Point2D& center) {
            map_.clean_dist();
            for(int x=0; x<divided_map_.width(); x++)
                for (int y=0;y<divided_map_.height(); y++) 
                    if (map_.get(x,y) == MAP_CELL_EMPTY)
                        investigate_region(track_map, track_stars, center, x, y);
        }
        
        void fix_missed_single_pixels() {
            for(int x=0; x<divided_map_.width(); x++)
                for (int y=0;y<divided_map_.height(); y++) 
                    if (divided_map_.get(x,y) == MAP_CELL_EMPTY) {
                        int new_val = 0;
                        for (int n=0;n<8; n++) {
                           if (divided_map_.get(x+NEIGHBOURS[n][0],y+NEIGHBOURS[n][1]) >=0 )
                               new_val = divided_map_.get(x+NEIGHBOURS[n][0],y+NEIGHBOURS[n][1]);
                        }
                        divided_map_.set(x,y, new_val);
                        DEBUG_MSG("Missed single pixel =("<<x<<","<<y<<")"<<endl);
                    }
        }
        
        void divide_by_extermals(Point2D& center, MapIf<int>& track_map) {
            DEBUG_MSG("dividing by EXTREMALS");
            cur_prob_ = 0.0;
            cur_region_ = 0;
            Galaxy glx = calculate_distances(map_, center.x, center.y, *pdf_, optimal_probs_, divided_map_, track_map);
            divided_map_.clean_dist();
            //DEBUG_IT(dist_map);
            DEBUG_IT(track_map, track_map);
            DEBUG_IT(stars, glx.track_stars);
            
            fix_individual_regions(track_map, glx.track_stars, center);
            
            DEBUG_IT(track_map, track_map);
            DEBUG_IT(stars, glx.track_stars);
            actual_probs_ = vector<float>(optimal_probs_.size(), 0.0);
            Point2D border_px = findBorderPoint(center);

                        //brd                 cnt                           oct bstar
            w_ = {{border_px.x, border_px.y}, {border_px.x-1, border_px.y}, 4,  -10};
            walkPetal(track_map, glx.track_stars, 0, false);

            fix_missed_single_pixels();
            
            //walkBorder(track_map, glx.track_stars, border_px);
            actual_probs_[cur_region_] = cur_prob_;
            cur_prob_ = 0;
            cur_region_=0;
            map_.clean_dist();
        }
        
        bool need_redivide() {
            for (int r=0; r<actual_probs_.size(); r++)
                if (actual_probs_[r]<0.1)
                    return true;
            return false;
        }
        
        void divide_mixed2(Point2D& center, MapIf<int>& track_map) {
            divide_by_extermals(center, track_map);
            if (need_redivide()) {
                DEBUG_MSG("REDIVISION 111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111");
                divide_by_equidists(center, track_map);
                if (need_redivide()) {
                    DEBUG_MSG("REDIVISION 22222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222")
                    divide_by_extermals(center, track_map);
                }
            }
        }

        
        /* +------+-------------+
         * |      |             | 
         * |      |      2      |
         * |  0   |-------------|
         * |      |             |
         * |------|      3      |
         * |  1   |             |
         * +------+-------------+  */
        void divide_vert_tiles() {
            ///measuring
            actual_probs_ = vector<float>(4, 0.0);
            float p=0.0, g, g_prev=std::numeric_limits<float>::max(), g_opt = optimal_probs_[0]+optimal_probs_[1];
            int x, y, div1, div2, div3;
            for(x=0;x<map_.width(); x++) {
                for (y=0; y<map_.height(); y++) 
                    if (map_.get(x,y) != MAP_CELL_OCCUPIED)
                        p += pdf_->get(x,y);
                g = p;
                if (fabs(g-g_opt)>fabs(g_prev-g_opt)) break;
                g_prev = g;
            }
            div1 = x-1;
            
            float p_total = g_prev;
            g_opt = p_total*optimal_probs_[0]/(optimal_probs_[0]+optimal_probs_[1]);
            p=0;
            g=0;
            g_prev = std::numeric_limits<float>::max();
            DEBUG_MSG("g_opt="<<g_opt<<", g_prev="<<g_prev<<", p_total="<<p_total);
            for (y=0;y<map_.height(); y++) {
                for(x=0; x<=div1; x++)
                    if (map_.get(x,y) != MAP_CELL_OCCUPIED)
                        p += pdf_->get(x,y);
                g = p;
                if (fabs(g-g_opt)>fabs(g_prev-g_opt)) break;
                g_prev = g;
            }
            div2 = y-1;
            
            p_total = 1-p_total;
            g_opt = p_total*optimal_probs_[2]/(optimal_probs_[2]+optimal_probs_[3]);
            p=0;
            g=0;
            g_prev = std::numeric_limits<float>::max();
            DEBUG_MSG("g_opt="<<g_opt<<", g_prev="<<g_prev<<", p_total="<<p_total);
            for (y=0;y<map_.height(); y++) {
                for(x=div1+1; x<map_.width(); x++)
                    if (map_.get(x,y) != MAP_CELL_OCCUPIED)
                        p += pdf_->get(x,y);
                g = p;
                if (fabs(g-g_opt)>fabs(g_prev-g_opt)) break;
                g_prev = g;
            }
            div3 = y-1;
            DEBUG_MSG("div1="<<div1<<", div2="<<div2<<", div3="<<div3);
                    
            ///dividing
            for (x=0;x<map_.width(); x++)
                for(y=0; y<map_.height(); y++)
                    if (map_.get(x,y) != MAP_CELL_OCCUPIED) {
                        if (x<=div1)
                            divided_map_.set(x,y, 0+(y>div2));
                        else
                            divided_map_.set(x,y, 2+(y>div3));
                    }                    
        }
        
        /* +------+-------------+
         * |           |        | 
         * |     0     |    1   |
         * |------+----+--------|
         * |      |             |
         * |  2   |      3      |
         * |      |             |
         * +------+-------------+  */
        void divide_horiz_tiles() {
            ///measuring
            actual_probs_ = vector<float>(4, 0.0);
            float p=0.0, g, g_prev=std::numeric_limits<float>::max(), g_opt = optimal_probs_[0]+optimal_probs_[1];
            int x, y, div1, div2, div3;
            for(y=0;y<map_.height(); y++) {
                for (x=0; x<map_.width(); x++) 
                    if (map_.get(x,y) != MAP_CELL_OCCUPIED)
                        p += pdf_->get(x,y);
                g = p;
                if (fabs(g-g_opt)>fabs(g_prev-g_opt)) break;
                g_prev = g;
            }
            div1 = y-1;
            
            float p_total = g_prev;
            g_opt = p_total*optimal_probs_[0]/(optimal_probs_[0]+optimal_probs_[1]);
            p=0;
            g=0;
            g_prev = std::numeric_limits<float>::max();
            DEBUG_MSG("g_opt="<<g_opt<<", g_prev="<<g_prev<<", p_total="<<p_total);
            for (x=0;x<map_.width(); x++) {
                for(y=0; y<=div1; y++)
                    if (map_.get(x,y) != MAP_CELL_OCCUPIED)
                        p += pdf_->get(x,y);
                g = p;
                if (fabs(g-g_opt)>fabs(g_prev-g_opt)) break;
                g_prev = g;
            }
            div2 = x-1;
            
            p_total = 1-p_total;
            g_opt = p_total*optimal_probs_[2]/(optimal_probs_[2]+optimal_probs_[3]);
            p=0;
            g=0;
            g_prev = std::numeric_limits<float>::max();
            DEBUG_MSG("g_opt="<<g_opt<<", g_prev="<<g_prev<<", p_total="<<p_total);
            for (x=0;x<map_.width(); x++) {
                for(y=div1+1; y<map_.height(); y++)
                    if (map_.get(x,y) != MAP_CELL_OCCUPIED)
                        p += pdf_->get(x,y);
                g = p;
                if (fabs(g-g_opt)>fabs(g_prev-g_opt)) break;
                g_prev = g;
            }
            div3 = x-1;
            DEBUG_MSG("div1="<<div1<<", div2="<<div2<<", div3="<<div3);
            
            ///dividing
            for (x=0;x<map_.width(); x++)
                for(y=0; y<map_.height(); y++)
                    if (map_.get(x,y) != MAP_CELL_OCCUPIED) {
                        if (y<=div1)
                            divided_map_.set(x,y, 0+(x>div2));
                        else
                            divided_map_.set(x,y, 2+(x>div3));
                    }
        }
        
        void divide(DivisionPolicy policy, vector<float>& optimal_probs, Point2D& center, MapIf<float>* pdf, MapIf<int>& track_map) { 
            pdf_ = pdf;
            optimal_probs_ = optimal_probs;
            switch(policy) {
                case(VERT_TILES)   : divide_vert_tiles(); break;
                case(HORIZ_TILES)  : divide_horiz_tiles(); DEBUG_MSG("------");break;
                case(BY_EXTREMALS) : divide_by_extermals(center, track_map); break;
                case(MIXED_METHOD1): divide_by_extermals(center, track_map); 
                case(BY_EQUIDISTS) : divide_by_equidists(center, track_map); break;
                case(MIXED_METHOD2): divide_mixed2(center, track_map); break;
                case(WRONG)        : break;
            }
            DEBUG_MSG("--------------------actual probabilities: [" << actual_probs_[0] << ", " << actual_probs_[1] << ", " << actual_probs_[2] << ", " << actual_probs_[3] << "]");
            count_ ++;
        }

};
const int MapDivider::NEIGHBOURS[8][2] = {{1,0}, {1,1}, {0,1}, {-1,1}, {-1,0}, {-1,-1}, {0,-1}, {1,-1}};
const int MapDivider::DIFF[2][8][2] = { 
            {{1,-1}, {1,0}, {1,1}, {0,1}, {-1,1}, {-1,0}, {-1,-1}, {0,-1}},
            {{0,-1}, {1,0}, {1,0}, {0,1}, {0,1},  {-1,0}, {-1,0},  {0,-1}}};
}
