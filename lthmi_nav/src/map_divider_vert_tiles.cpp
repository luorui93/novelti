#define TRACK_MAP 1
#define CIRC_DIV 1
#include "fast_dist.cpp"

#include <math.h>
#include <cmath>

namespace lthmi_nav {

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
        
            
        struct BorderWalker {
            Point2D brd;
            Point2D cnt;
            char oct;            
            int bstar;
        };
        
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

        
        void repaintPixel(int x, int y, MapIf<int>& src_map, MapIf<int>& dest_map, int old_val, int new_val) {
            if (src_map.get(x,y) == old_val) 
                dest_map.set(x,y, new_val);
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
        

        
        void divide(vector<float>& optimal_probs, Point2D& center, MapIf<float>* pdf, MapIf<int>& track_map) { 
            pdf_ = pdf;
            optimal_probs_ = optimal_probs;
            divide_vert_tiles();
            DEBUG_MSG("--------------------actual probabilities: [" << actual_probs_[0] << ", " << actual_probs_[1] << ", " << actual_probs_[2] << ", " << actual_probs_[3] << "]");
            count_ ++;
        }

};
}
