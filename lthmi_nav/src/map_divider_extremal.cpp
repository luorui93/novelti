        /* +--------------------------------+
         * |                                |
         * |                                |
         * |                                |
         * |        _____________           |
         * |      _/             \_         |
         * |     /    _________    \        |
         * |    |    /         \    \       |
         * |   /    /   ____    \    \      |
         * |  |    |   /    \   |    |      |
         * |  |    |  |  0   | 1|  2 |   3  |
         * |  |    |   \____/   |    |      |
         * |  \     \          /     /      |
         * |   \     \________/     |       |
         * |    \_                _/        |
         * |      \______________/          |
         * |                                |
         * +--------------------------------+  */
        
#include <lthmi_nav/map_divider.h>
#include <CompoundMap.h>
#include <CWave2.h>

using namespace cwave;

namespace lthmi_nav {


/*
class CWaveProc2: public CWave2Processor {
public:
    IntMap& map_divided;
    FloatMapConstPtr& pdf;
    std::vector<double>& probs_optimal;
    std::vector<double>& probs_actual;
    int region;
    double prob;
    
    CWaveProc2(IntMap& map_divided1, FloatMapConstPtr& pdf1, std::vector<double>& probs_optimal1, std::vector<double>& probs_actual1):
        map_divided(map_divided1),
        pdf(pdf1),
        probs_optimal(probs_optimal1),
        probs_actual(probs_actual1)
    {
        region = 0;
        prob = 0.0;
    }
    
    void visitVertex(Point& pt) {
        double p = pdf->data[pt.x + pt.y*pdf->info.width];
        prob += p;
        map_divided.data[pt.x + pt.y*map_divided.info.width] = region;
        if ( prob >= probs_optimal[region] ) {
            probs_actual[region] = prob;
            prob = 0.0;
            region++;
        }
    }
    
    void onInitSource(Point& pt) {
        visitVertex(pt);
    }
    
    void onSetPointDistance(Star& star, OctPoint& op, bool is_nbp, Point& pt, int old_dist, int new_dist) {
        if (old_dist==MAP_POINT_UNEXPLORED) {
            visitVertex(pt);
        }
    }

    void onAddStar(Star& s) {};
    void onDistanceCorrection(Point& pt, int old_dist, int new_dist) {
        if (old_dist==MAP_POINT_UNEXPLORED) {
            visitVertex(pt);
        }
    };
    
    void finish() {
        probs_actual[region] = prob;
    }
};*/



class ExtremalMapDivider :  public MapDivider, public CWave2Processor {
public:
    CompoundMap cmap;
    std::vector<int> track_stars;
    IntMap track_map;
    int region;
    double prob;
    
    
    ExtremalMapDivider() :
        MapDivider() 
    { }
    
    void start(lthmi_nav::StartExperiment::Request& req) {
        new (&cmap) CompoundMap(req.map.info.width, req.map.info.height);
        for (int x=0; x<req.map.info.width; x++)
            for (int y=0; y<req.map.info.height; y++)
                if (req.map.data[x + y*req.map.info.width]==0)
                    cmap.setPixel(x,y, FREED); //free
        MapDivider::start(req);
    }
    

    
    void divide() {
        beforeCWave();
        CWave2 cw(cmap);
        cw.setProcessor(this);
        Point center(vx.x,vx.y);
        cw.calc(center);
        afterCwave();
        //v = findBoundaryVertex(center);

        cmap.clearDist();
        //ROS_INFO("probs_actual=[%f,%f,%f,%f]", probs_actual[0], probs_actual[1], probs_actual[2], probs_actual[3]);
    }
    
    /*Point findBoundaryVertex(Point pt) {
        do {
        } while (cmap.
    }*/
        
        
    void visitVertex(Point& pt) {
        double p = pdf->data[pt.x + pt.y*pdf->info.width];
        prob += p;
        map_divided.data[pt.x + pt.y*map_divided.info.width] = region;
        if ( prob >= probs_optimal[region] ) {
            probs_actual[region] = prob;
            prob = 0.0;
            region++;
        }
    }
    
    void onInitSource(Point& pt) {
        visitVertex(pt);
    }
    
    void onSetPointDistance(Star& star, OctPoint& op, bool is_nbp, Point& pt, int old_dist, int new_dist) {
        if (old_dist==MAP_POINT_UNEXPLORED) {
            visitVertex(pt);
        }
    }

    void onAddStar(Star& s) {};
    void onDistanceCorrection(Point& pt, int old_dist, int new_dist) {
        if (old_dist==MAP_POINT_UNEXPLORED) {
            visitVertex(pt);
        }
    };
    
    void beforeCWave() {
        region = 0;
        prob = 0.0;
    }
    
    void afterCwave() {
        probs_actual[region] = prob;
    }
        
};
}
