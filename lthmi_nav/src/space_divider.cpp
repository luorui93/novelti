#include "ros/ros.h" //for debug only
#define TRACK_MAP 1
#include "fast_dist.cpp"

#include <lthmi_nav/space_divider.h>
#include <math.h>
#include <cmath>
#include <ncurses.h>

#ifdef DEBUG
    #include <stdlib.h>  //needed for debug only
#endif


#ifdef COLORED
    #define HIGHLIGHT_(x,y,color) sd.map.highlight((x),(y),(color)); 
#else
    #define HIGHLIGHT_(x,y,color) 
#endif

#ifdef DEBUG
    #define HIGHLIGHT(x,y,color) HIGHLIGHT_((x),(y),(color)); sd.map.display();
#else
    #define HIGHLIGHT(x,y,color) HIGHLIGHT_((x),(y),(color));
#endif


#include "geometry_msgs/PoseStamped.h"
//ros::Publisher* lthmi_nav::pub_debug_pose;


namespace lthmi_nav {
    
    Point2D* map_divide(
            vector<float> opt_probs, 
            MapIf<int>& map, 
            MapIf<float>& pdf, 
            Point2D& c,
            MapIf<int>& track_map, 
            MapIf<int>& divided_map//,
            //ros::Publisher* pub_debug_pose
    ) {
        //ROS_INFO("space_divider_node: =========== (10,6)=>%d", divided_map.get(10,6));
        Galaxy glx = calculate_distances(map, c.x, c.y, track_map);
        Point2D* path_end_point = divide(opt_probs, map, pdf, track_map, divided_map, glx.track_stars, c);//, pub_debug_pose);
        map.clean_dist();
        return path_end_point;
    } 
    
    Point2D* divide(
            vector<float> opt_probs, 
            MapIf<int>& map, 
            MapIf<float>& pdf, 
            MapIf<int>& track_map, 
            MapIf<int>& divided_map, 
            vector<Point2D>& track_stars, 
            Point2D center//,
            //ros::Publisher* pub_debug_pose
    ) {
        vector<float> opt_prob_threshs = opt_probs;
        for(int k=1; k<opt_prob_threshs.size(); k++)
            opt_prob_threshs[k] += opt_prob_threshs[k-1];
        ROS_INFO("space_divider: opt_prob_threshs = (%f,%f,%f,%f)", 
                 opt_prob_threshs[0], opt_prob_threshs[1], opt_prob_threshs[2], opt_prob_threshs[3]);
        SpaceDivider sd = {
            map, 
            pdf,
            track_map,
            track_stars, 
            {0}, //walker
            divided_map, 
            opt_prob_threshs,
            0.0, // cur_prob;pi
            0,    // cur_region;
            //pub_debug_pose
        };
        //ROS_INFO("space_divider_node: ===================== (10,6)=>%d", divided_map.get(10,6));
        Point2D p = find_border_point(sd, center);
        ROS_INFO("space_divider: border point = (%d,%d)", p.x,p.y);
                   //x    y    oct  dist                cur_star
        sd.walker = {p.x, p.y, 0,   map.get(p.x-1,p.y), 0};
        do {
            border_step(sd, sd.walker);
        } while (!(sd.walker.x==p.x && sd.walker.y==p.y && sd.walker.oct==0));
        Point2D* path_end_point = NULL;
        return path_end_point;
    }
    
    void publish_debug_pose(SpaceDivider &sd, int x, int y) {
        geometry_msgs::PoseStamped p;
        p.header.frame_id="/map";
        double resolution = 0.1;
        p.pose.position.x = resolution*x;
        p.pose.position.y = resolution*y;
        pub_debug_pose->publish(p);
        ros::Duration(0.01).sleep();
    }
    
    Point2D find_border_point(SpaceDivider &sd, Point2D& p) {
        int x=p.x, c = sd.map.get(x,p.y);
        for (int c2=sd.map.get(++x,p.y); c2>c; c=c2, c2 = sd.map.get(++x,p.y))
            pixel_process(sd, x, p.y);
        return {x,p.y};
    }

    #define border_step_macro(dx, dy, oct_occ, oct_free)\
        x = w.x+(dx); y=w.y+(dy);\
        star = sd.track_map.get(x,y);\
        if (star!=w.cur_star) {\
            if (star<0 || (sd.track_map.get(sd.track_stars[star].x, sd.track_stars[star].y) != w.cur_star && \
                           sd.track_map.get(sd.track_stars[w.cur_star].x, sd.track_stars[w.cur_star].y) != star)) {\
                w.x=x; w.y=y; w.oct=oct_occ; break;\
            }\
        } \
        w.cur_star=star; w.oct=oct_free; \
        path_walk(sd, x, y); break\
        
        
    void border_step(SpaceDivider& sd, BorderWalker& w) {
        int d,y,x,star;
        //publish_debug_pose(sd,w.x,w.y);
        /*Point2D p;
        /**/
        switch(w.oct) {
            case 0: border_step_macro(-1, +1, 2, 7); 
            case 1: border_step_macro(-1,  0, 2, 0); 
            case 2: border_step_macro(-1, -1, 4, 1); 
            case 3: border_step_macro( 0, -1, 4, 2); 
            case 4: border_step_macro(+1, -1, 6, 3); 
            case 5: border_step_macro(+1,  0, 6, 4); 
            case 6: border_step_macro(+1, +1, 0, 5); 
            case 7: border_step_macro( 0, +1, 0, 6); 
        } /**/
    }
    

    
    void path_walk(SpaceDivider& sd, int x, int y) {
        if (sd.cur_prob>=sd.opt_prob_threshs[sd.cur_region] && sd.cur_region!=sd.opt_prob_threshs.size()-1) {
            sd.cur_region++;
        }
        ///sd.track_map.highlight(x, y, PIXEL_BORDER); 
        ///sd.track_map.display(sd.track_map.height()+5, 0);
        
        int parent_id = sd.track_map.get(x,y);
        Point2D s = {x,y};
        Point2D e = sd.track_stars[parent_id];
        line_walk(sd, s, e);
        /*do {
            star_id = sd.track_map[x][y];
            Point2D e = stars[star_id];
            line_walk(s, e);
        } while (star_id!=0);*/
    }
    
    
    //TODO optimize: process first pixel, don't process last pixel
    #define line_loop(xx, yy) \
        do {\
            if (leps>0) {leps -= dx; y++;}\
            leps += dy; x++;\
            pixel_process(sd, s.x+(xx), s.y+(yy));\
        } while (x<dx); \
        break;
    
    void line_walk(SpaceDivider &sd, Point2D s, Point2D e) {
        int dx = e.x-s.x, 
            dy = e.y-s.y,
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
        
        pixel_process(sd, s.x, s.y);
        switch(oct) {
            case 0: line_loop( x,  y); 
            case 1: line_loop( y,  x); 
            case 2: line_loop(-y,  x); 
            case 3: line_loop(-x,  y); 
            case 4: line_loop(-x, -y); 
            case 5: line_loop(-y, -x); 
            case 6: line_loop( y, -x); 
            case 7: line_loop( x, -y); 
        }/**/
    }
    

    
    void pixel_process(SpaceDivider &sd, int x, int y) {
        int reg = sd.divided_map.get(x,y);
        //publish_debug_pose(sd,x,y);
        //ROS_INFO("space_divider: (%d,%d), reg=%d  set to %d", x,y,reg, sd.cur_region);
        if (reg==-1) {
            //sd.map.highlight(x,y, (sd.cur_region%5)+PIXEL_REGION); sd.map.display();
            sd.divided_map.set(x,y, sd.cur_region);
            sd.cur_prob += sd.pdf.get(x,y);
            ///ROS_INFO("space_divider: (%d,%d), cur_prob=%f, cur_region=%d", x,y, sd.cur_prob, sd.cur_region);
        }
    }
    
}
