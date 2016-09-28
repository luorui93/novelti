#include <math.h>  
#include <map>
#include <low_throughput_hmi/map.h>

using namespace low_throughput_hmi_cost;
using namespace std;


map<const char*, vector<Wall> >  generate_test_mazes(int width, int height) {
    double W=double(width), H=double(height), CX=W/2, CY=H/2;
    
    map<const char*, vector<Wall> > mazes;

    mazes["4walls"] = {   {0,0,W}, {0,H-1,W}, {0,0,-H}, {W-1,0,-H}    };
    mazes["1pt"] = {   {0,0,W}, {0,H-1,W}, {0,0,-H}, {W-1,0,-H},     {CX+2,CY+1,1}    };
    mazes["pi"] = {
            {0,0,W}, {0,H-1,W}, {0,0,-H}, {W-1,0,-H}, //4 walls
            {W/4, H/4, W/4+1}, {W/4, 3*H/4, W/4+1}, {W/2, H/4, -(H/2+1)} // =|
        };
    
        
    double r1w=0.2*W, r1h=0.15*H, r2w=0.3*W, r2h=0.2*H,
           r3x=0.2*W, r3y=0.3*H, r3w=0.6*W, r3h=0.5*H, d=0.07*W;
    mazes["4rooms"] = {
            {0,0,W}, {0,H-1,W}, {0,0,-H}, {W-1,0,-H}, //4 walls
            {0,r1h,(r1w-d)/2}, {(r1w+d)/2,r1h,(r1w-d)/2+2}, {r1w,0,-r1h}, //room 1
            {W-r2w,0,-(r2h-d)/2}, {W-r2w,(r2h+d)/2, -(r2h-d)/2-1}, {W-r2w,r2h,r2w}, //room 2
            /*{r3x, r3y+r3h/2, r3w},*/ {r3x,r3y,-r3h},{r3x+r3w,r3y,-r3h},{r3x,r3y+r3h,r3w-d},//room 3
            {r3x, r3y, (r3w-d)/2}, {r3x+(r3w+d)/2+1, r3y, (r3w-d)/2},//room 4
        };
    mazes["4rooms2"] = {
            {0,0,W}, {0,H-1,W}, {0,0,-H}, {W-1,0,-H}, //4 walls
            {0,r1h,(r1w-d)/2}, {(r1w+d)/2,r1h,(r1w-d)/2+2}, {r1w,0,-r1h}, //room 1
            {W-r2w,0,-(r2h-d)/2}, {W-r2w,(r2h+d)/2, -(r2h-d)/2-1}, {W-r2w,r2h,r2w}, //room 2
            {r3x, r3y+r3h/2, r3w/2-1}, {r3x+r3w/2, r3y+r3h/2, r3w/2}, {r3x,r3y,-r3h},{r3x+r3w,r3y,-r3h},{r3x,r3y+r3h,r3w-d},//room 3
            {r3x, r3y, (r3w-d)/2}, {r3x+(r3w+d)/2+1, r3y, (r3w-d)/2},//room 4
        };
    
    //W=100;
    d=0.07*H;
    r3x=0.2*H; r3y=0.3*H+30; r3w=0.6*H;
    mazes["4rooms3"] = {
            {0,0,W}, {0,H-1,W}, {0,0,-H}, {W-1,0,-H}, //4 walls
            {r3x, r3y, (r3w-d)/2}, {r3x+(r3w+d)/2+1, r3y, (r3w-d)/2},//room 4
        };

    
    //maze=6
    /*double s=0.2*W;
    vector<double> rx={0.1*W, 0.4*W, 0.7*W,   0.1*W, 0.4*W, 0.7*W,   0.1*W, 0.4*W, 0.7*W,   };
    vector<double> ry={0.1*W, 0.1*W, 0.1*W,   0.4*W, 0.4*W, 0.4*W,   0.7*W, 0.7*W, 0.7*W,   }; */
    double s=0.3*W;
    vector<double> rx={round(0.15*W), round(0.6*W),  round(0.15*W), round(0.6*W),   };
    vector<double> ry={round(0.15*W), round(0.15*W),  round(0.6*W), round(0.6*W),   };
    
    mazes["rooms_with_1px_doors"] = {
            {0,0,W}, {0,H-1,W}, {0,0,-H}, {W-1,0,-H}, //4 walls
    };
    for (int r=0; r<rx.size(); r++) {
        vector<Wall> room = {
            {rx[r], ry[r], s/2},   {rx[r]+s/2+1, ry[r], s/2},   {rx[r], ry[r], -s/2},   {rx[r], ry[r]+s/2+1, -s/2}, //room1
            {rx[r], ry[r]+s, s/2}, {rx[r]+s/2+1, ry[r]+s, s/2}, {rx[r]+s, ry[r], -s/2}, {rx[r]+s, ry[r]+s/2+1, -(1+s/2)}
        };
        for (int k=0; k<room.size(); k++)
            mazes["rooms_with_1px_doors"].push_back(room[k]);
    };
   
    
    mazes["multi_peak"] = {
            {0,0,5}, {0,9-1,5}, {0,0,-9}, {5-1,0,-9}, //4 walls
            {2, 2, -5} // |
        };
    
    
    return mazes;
};
    
