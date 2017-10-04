#include "ros/ros.h"

#define MEAN_DIST 1
#define PDF_STAT 1
#include "fast_dist.cpp"

#include <novelti/map_if.h>

#include <stdexcept>

namespace novelti {


#ifdef DEBUG_POSE_FINDER
class OptimalPoseFinderDebugger {
    public:
        std::ostringstream msg_stream_;
        virtual ~OptimalPoseFinderDebugger() {};
        virtual void text() {};
        virtual void mean_dist_map() {};
        virtual void pose(int x, int y) {};
};
#endif
    
class OptimalPoseFinder {
    
    public:
        static const float REACHABLE_CELL;
        static const float UNREACHABLE_CELL;
        static const int NEIGHBOURS[8][2];
        
        enum Method {
            OPT,
            LOCALOPT,
            MAXPROB,
            COG,
            TOMAXPROB,
            COG2,
            COG2LOPT,
            ALL3COG,
            ALL3GOPT,
            ALL3LOPT
        };
        
        #ifdef DEBUG_POSE_FINDER
            OptimalPoseFinderDebugger* debugger_;
            #define DEBUG_MSG(my_stream) debugger_->msg_stream_ << my_stream; debugger_->text(); debugger_->msg_stream_.str(""); debugger_->msg_stream_.clear();
            #define DEBUG_IT(type, ...) debugger_->type(__VA_ARGS__);
        #else
            #define DEBUG_MSG(...)
            #define DEBUG_IT(type, ...)
        #endif
        
        
        
        MapIf<int>& map_;
        MapIf<float>& reach_;
        int max_dist_;
        Method method_;
        
        Point2D reach_to_main_;
        
        int runs_;
        
        OptimalPoseFinder(Method method, MapIf<int>& map, int max_dist, MapIf<float>& reach) :
            map_(map),
            reach_(reach) 
        { 
            max_dist_ = max_dist; 
            method_ = method;
        }
        
        void calcReachabilityArea(Point2D& cur_pose, MapIf<float>& pdf) {
            reach_to_main_ = {cur_pose.x-max_dist_, cur_pose.y-max_dist_};
            reach_.resize(2*max_dist_+1, 2*max_dist_+1, UNREACHABLE_CELL);
            
            calculate_distances(map_, cur_pose.x, cur_pose.y, 2*max_dist_, pdf); //TODO remove pdf
            
            int x_min = max(0, reach_to_main_.x);
            int y_min = max(0, reach_to_main_.y);
            int x_max = min(map_.width()-1,  cur_pose.x+max_dist_);
            int y_max = min(map_.height()-1, cur_pose.y+max_dist_);

            for (int x=x_min; x<=x_max; x++)
                for (int y=y_min; y<=y_max; y++)
                    if (map_.get(x,y)>=0) {
                        map_.set(x,y,   MAP_CELL_EMPTY); //cleaning up map
                        reach_.set(x-reach_to_main_.x, y-reach_to_main_.y,   REACHABLE_CELL);
                    } 
            DEBUG_MSG("Reachability area size is "<<reach_.width()<<"x"<<reach_.height());
        }
        
        float calcMeanDistanceInReachArea(Point2D pose, MapIf<float>& pdf) {
            runs_++;
            Galaxy glx = calculate_distances(map_, pose.x+reach_to_main_.x, pose.y+reach_to_main_.y, -1, pdf);
            float qq=0.0;
            /*for (int y=0; y< map_.height(); y++)
            for (int x=0; x< map_.width(); x++)
                    if (map_.get(x,y)>=0)
                        qq += pdf.get(x,y)*map_.get(x,y);/**/
            //DEBUG_MSG(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> pdf_stat="<<glx.pdf_stat<<", qq="<<qq);
            //DEBUG_MSG(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>pose=("<<pose.x+reach_to_main_.x<<","<<pose.y+reach_to_main_.y<<")=> pdf_stat="<<glx.pdf_stat<<",       runs="<<runs_);
            //if (fabs(qq-glx.pdf_stat)>0.000001)
                //throw std::invalid_argument( "**********************************************************" );
            map_.clean_dist();
            return glx.pdf_stat;
        }
        
        
        
        float calcMeanDistance(Point2D pose, MapIf<float>& pdf) {
            runs_++;
            Galaxy glx = calculate_distances(map_, pose.x, pose.y, -1, pdf);
            float qq=0.0;
            /*for (int y=0; y< map_.height(); y++)
            for (int x=0; x< map_.width(); x++)
                    if (map_.get(x,y)>=0)
                        qq += pdf.get(x,y)*map_.get(x,y);/**/
            //DEBUG_MSG(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> pdf_stat="<<glx.pdf_stat<<", qq="<<qq);
            //DEBUG_MSG(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>pose=("<<pose.x+reach_to_main_.x<<","<<pose.y+reach_to_main_.y<<")=> pdf_stat="<<glx.pdf_stat<<",       runs="<<runs_);
            //if (fabs(qq-glx.pdf_stat)>0.000001)
                //throw std::invalid_argument( "**********************************************************" );
            map_.clean_dist();
            return glx.pdf_stat;
        }
        
        void calcMeanDistMap(MapIf<float>& pdf) {
            reach_.resize(map_.width(), map_.height(), UNREACHABLE_CELL);
            for (int x=0; x<map_.width(); x++)
                for (int y=0; y<map_.height(); y++) {
                    if (map_.get(x,y)!=MAP_CELL_OCCUPIED) {
                        DEBUG_MSG("x,y=("<<x<<","<<y<<")");
                        reach_.set(x,y,   calcMeanDistance({x,y}, pdf));
                    }
                }
            DEBUG_IT(mean_dist_map);
        }
        
        
        
        
        Point2D find(Point2D& cur_pose,  MapIf<float>& pdf) {
            //calcMeanDistMap(pdf);
            runs_=0;
            switch(method_) {
                case(OPT)      : return findGlobalMinimum(cur_pose, pdf);
                case(LOCALOPT) : return findLocalMinimum(cur_pose, pdf); 
                case(MAXPROB)  : return findMaxProb(cur_pose, pdf); 
                case(TOMAXPROB): return findToMaxProb(cur_pose, pdf); 
                case(COG)      : return findCOG(cur_pose, pdf); 
                case(COG2)     : return findCOG2(cur_pose, pdf); 
                case(COG2LOPT) : return findCOG2LocalMin(cur_pose, pdf); 
                case(ALL3COG)  : return findAll3(cur_pose, pdf);
                case(ALL3LOPT) : return findAll3(cur_pose, pdf);
                case(ALL3GOPT) : return findAll3(cur_pose, pdf);
            }
        }
        
        Point2D findAll3(Point2D& cur_pose,  MapIf<float>& pdf) {
            DEBUG_MSG("Going to calculate ALL THREE 'best' poses");
            
            Point2D cog2_pose = findCOG2(cur_pose,pdf); //wrt map
            Point2D pose = {cog2_pose.x-reach_to_main_.x, cog2_pose.y-reach_to_main_.y}; // wrt reach area frame
            float opt_mean_dist = slideToMinInReachArea(pose, pdf); // wrt reach area frame
            pose.x += reach_to_main_.x;
            pose.y += reach_to_main_.y;
            Point2D cog2lopt_pose = pose;
            float cog2_min_dist = reach_.get(cog2_pose.x-reach_to_main_.x,cog2_pose.y-reach_to_main_.y);
            float cog2lopt_min_dist = reach_.get(cog2lopt_pose.x-reach_to_main_.x,cog2lopt_pose.y-reach_to_main_.y);
            
            Point2D globopt_pose =findGlobalMinimum(cog2lopt_pose, pdf);
            DEBUG_MSG(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> cog2_pose    =("<<cog2_pose.x<<","<<cog2_pose.y<<"), mean dist="<<cog2_min_dist);
            DEBUG_MSG(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> cog2lopt_pose=("<<cog2lopt_pose.x<<","<<cog2lopt_pose.y<<"), mean dist="<<cog2lopt_min_dist);
            DEBUG_MSG(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> globopt_pose =("<<globopt_pose.x<<","<<globopt_pose.y<<"), mean dist="<<reach_.get(globopt_pose.x-reach_to_main_.x,globopt_pose.y-reach_to_main_.y));
            switch(method_) {
                case(ALL3COG)  : return cog2_pose;
                case(ALL3LOPT) : return cog2lopt_pose;
                case(ALL3GOPT) : return globopt_pose;
            }
        }
        
        Point2D findMaxProb(Point2D& cur_pose,  MapIf<float>& pdf) {
            DEBUG_MSG("Looking for pixel with MAXIMUM PROBABILITY");
            calcReachabilityArea(cur_pose, pdf);
            Point2D pose;
            float p, max_prob = 0.0;
            for (int x=0; x<reach_.width(); x++)
                for (int y=0; y<reach_.height(); y++) {
                    if (reach_.get(x,y) != UNREACHABLE_CELL) {
                        p = pdf.get(x+reach_to_main_.x,y+reach_to_main_.y);
                        //DEBUG_MSG("pdf("<<x+reach_to_main_.x<<","<<y+reach_to_main_.y<<")="<<p);
                        if (p>max_prob) {
                            max_prob = p;
                            DEBUG_MSG("    better pt=("<<x+reach_to_main_.x<<","<<y+reach_to_main_.y<<"), p="<<max_prob);
                            pose = {x,y}; // wrt reach area frame
                        }
                    }
                }
            pose.x += reach_to_main_.x;
            pose.y += reach_to_main_.y;
            return pose; // now wrt map frame
        }
        
        Point2D findToMaxProb(Point2D& cur_pose,  MapIf<float>& pdf) {
            DEBUG_MSG("Looking for pixel in reachability area CLOSEST to the MAXIMUM PROBABILITY pixel");
            calcReachabilityArea(cur_pose, pdf);
            Point2D pose;
            float p, max_prob = 0.0;
            for (int x=0; x<pdf.width(); x++)
                for (int y=0; y<pdf.height(); y++) {
                    p = pdf.get(x,y);
                    if (p>max_prob) {
                        max_prob = p;
                        pose = {x,y}; // wrt map frame
                    }
                }
            //now 'pose' points to a pixel on map with max probability, 
            //need to find a pixel in reachability area which is closest to this max prob pixel
            calculate_distances(map_, pose.x, pose.y, -1, pdf); //TODO remove pdf
            
            int dist, min_dist = std::numeric_limits<int>::max();
            for (int x=0; x<reach_.width(); x++)
                for (int y=0; y<reach_.height(); y++) {
                    if (reach_.get(x,y) != UNREACHABLE_CELL) {
                        dist = map_.get(x+reach_to_main_.x,y+reach_to_main_.y);
                        if (dist<min_dist) {
                            min_dist = dist;
                            pose = {x,y}; // wrt reach area frame
                        }
                    }
                }
            
            map_.clean_dist();
            pose.x += reach_to_main_.x;
            pose.y += reach_to_main_.y;
            return pose; // now wrt map frame
        }
        
        
        Point2D calcPdfCog(MapIf<float>& pdf) { //returns wrt map
            float p, xsum=0.0, ysum=0.0, psum=0.0;
            for (int x=0; x<pdf.width(); x++)
                for (int y=0; y<pdf.height(); y++) {
                    p = pdf.get(x,y);
                    if (p>0.0) {
                        xsum += p*x;
                        ysum += p*y;
                        psum += p;
                    }
                }
            return {int(round(xsum/psum)), int(round(ysum/psum))}; //wrt map_
        }
            
            
        Point2D findCOG(Point2D& cur_pose,  MapIf<float>& pdf) {
            DEBUG_MSG("Looking for pixel in reachability area CLOSEST to pdf COG");
            calcReachabilityArea(cur_pose, pdf);
            
            Point2D cog_pose = calcPdfCog(pdf);
            cog_pose.x -= reach_to_main_.x;   
            cog_pose.y -= reach_to_main_.y;   //now wrt reach area
            
            Point2D pose;
            int dist, min_dist = std::numeric_limits<int>::max();
            for (int x=0; x<reach_.width(); x++)
                for (int y=0; y<reach_.height(); y++) {
                    if (reach_.get(x,y) != UNREACHABLE_CELL) {
                        dist = sqrt((cog_pose.x-x)*(cog_pose.x-x) + (cog_pose.y-y)*(cog_pose.y-y));
                        if (dist<min_dist) {
                            min_dist = dist;
                            pose = {x,y}; // wrt reach area frame
                        }
                    }
                }
            
            pose.x += reach_to_main_.x;
            pose.y += reach_to_main_.y;
            return pose; // now wrt map frame
        }
        
        Point2D findCOG2(Point2D& cur_pose,  MapIf<float>& pdf) {
            DEBUG_MSG("Looking for pixel in reachability area CLOSEST to pdf COG");
            calcReachabilityArea(cur_pose, pdf);
            
            Point2D cog_pose = calcPdfCog(pdf);  //wrt to map

            //find point on map closest to COG (equalidian distance)
            Point2D pose;
            float dist, min_dist = std::numeric_limits<int>::max();
            for (int x=0; x<map_.width(); x++)
                for (int y=0; y<map_.height(); y++) {
                    if (map_.get(x,y) != MAP_CELL_OCCUPIED) {
                        dist = sqrt((cog_pose.x-x)*(cog_pose.x-x) + (cog_pose.y-y)*(cog_pose.y-y));
                        if (dist<min_dist) {
                            min_dist = dist;
                            pose = {x,y}; // wrt reach area frame
                        }
                    }
                }
            //find point in reachability area closest (obstacled distance) to pose
            calculate_distances(map_, pose.x, pose.y, -1, pdf); //TODO remove pdf
            int idist, imin_dist = std::numeric_limits<int>::max();
            for (int x=0; x<reach_.width(); x++)
                for (int y=0; y<reach_.height(); y++) {
                    if (reach_.get(x,y) != UNREACHABLE_CELL) {
                        idist = map_.get(x+reach_to_main_.x,y+reach_to_main_.y);
                        if (idist<0) { DEBUG_MSG("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa idist="<<idist);};
                        if (idist<imin_dist) {
                            imin_dist = idist;
                            pose = {x,y}; // wrt reach area frame
                        }
                    }
                }
            map_.clean_dist();
            
            pose.x += reach_to_main_.x;
            pose.y += reach_to_main_.y;
            return pose; // now wrt map frame            
        }
        
        Point2D findCOG2LocalMin(Point2D& cur_pose,  MapIf<float>& pdf) {
            Point2D pose = findCOG2(cur_pose,pdf);
            pose = {pose.x-reach_to_main_.x, pose.y-reach_to_main_.y}; // wrt reach area frame
            float opt_mean_dist = slideToMinInReachArea(pose, pdf); // wrt reach area frame
            pose.x += reach_to_main_.x;
            pose.y += reach_to_main_.y;
            return pose; // now wrt map frame
        }
            
        Point2D findLocalMinimum(Point2D& cur_pose,  MapIf<float>& pdf) {
            DEBUG_MSG("Looking for LOCAL min in reachability area");
            calcReachabilityArea(cur_pose, pdf);
            Point2D pose = {cur_pose.x-reach_to_main_.x, cur_pose.y-reach_to_main_.y}; // wrt reach area frame
            float opt_mean_dist = slideToMinInReachArea(pose, pdf); // wrt reach area frame
            pose.x += reach_to_main_.x;
            pose.y += reach_to_main_.y;
            return pose; // now wrt map frame
        }
        
        Point2D findGlobalMinimum(Point2D& cur_pose,  MapIf<float>& pdf) {
            DEBUG_MSG("Looking for GLOBAL min in reachability area");
            calcReachabilityArea(cur_pose, pdf);
            return findGlobalMinimumWithRA(cur_pose, pdf);
        }

        bool genPointInRA(Point2D& pose, int max_attempts) {
            int x,y, attempts=0, w=reach_.width(), h = reach_.height();
            do {
                x = rand() % w;
                y = rand() % h;
                attempts++;
                if (reach_.get(x,y) == REACHABLE_CELL) {
                    pose={x,y};
                    return true;
                }
            } while (attempts < max_attempts);
            return false;
        }
        
        Point2D findGlobalMinimumWithRA(Point2D& init_pose,  MapIf<float>& pdf) {
            int start_points=25;
            int max_attempts = 1000;
            Point2D pose = {init_pose.x-reach_to_main_.x, init_pose.y-reach_to_main_.y}; // wrt reach area frame
            float min_dist = std::numeric_limits<float>::max();
            Point2D opt_pose = pose;
            do {
                DEBUG_MSG("Trying start point reach_["<<pose.x<<","<<pose.y<<"]==" << reach_.get(pose.x,pose.y))
                float dist = slideToMinInReachArea(pose, pdf); // wrt reach area frame
                if (dist<min_dist) {
                    min_dist = dist;
                    opt_pose = pose;
                }
                start_points--;
            } while (start_points!=0 && genPointInRA(pose, max_attempts));
            // opt_pose is wrt reach area frame
            opt_pose.x += reach_to_main_.x;
            opt_pose.y += reach_to_main_.y;
            return opt_pose; // now wrt map frame
        }
        
        float slideToMinInReachArea(Point2D& p, MapIf<float>& pdf) { 
            /* moves p to optimal pose
             * returns mean distance in optimal pose */
            float dist, min_dist = calcMeanDistanceInReachArea(p, pdf);
            reach_.set(p.x,p.y,  min_dist);
            int best_nb;
            int x,y;
            
            while (true) {
                best_nb = -1; //best neighbour
                for (int nb=0; nb<8; nb++) { //iterate over neighbours
                    x = p.x+NEIGHBOURS[nb][0]; 
                    y = p.y+NEIGHBOURS[nb][1];
                    if (reach_.get(x, y)==REACHABLE_CELL) { //means not visited
                        dist = calcMeanDistanceInReachArea({x,y}, pdf);
                        reach_.set(x,y,  dist);
                        if ( dist < min_dist) {
                            min_dist = dist;
                            best_nb = nb;
                        }
                    }
                }
                if (best_nb>=0) {
                    p.x+=NEIGHBOURS[best_nb][0];
                    p.y+=NEIGHBOURS[best_nb][1];
                } else {
                    break;
                }
            } 
            return min_dist;
        }
}; //class

const int OptimalPoseFinder::NEIGHBOURS[8][2] = {{1,0}, {1,1}, {0,1}, {-1,1}, {-1,0}, {-1,-1}, {0,-1}, {1,-1}};
const float OptimalPoseFinder::REACHABLE_CELL =  -1.0;
const float OptimalPoseFinder::UNREACHABLE_CELL = -10.0;

} //namespace
