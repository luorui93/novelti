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



class EquidistMapDividerCWaveProc: public CWave2Processor {
public:
    MapDivider& mdiv_;
    
    EquidistMapDividerCWaveProc(MapDivider& mdiv):
        mdiv_(mdiv)
    {}
    
    void onInitSource(Point& pt) {
        mdiv_.markVertex(pt.x, pt.y);
    }
    
    void onSetPointDistance(Star& star, OctPoint& op, bool is_nbp, Point& pt, int old_dist, int new_dist) {
        mdiv_.markVertex(pt.x, pt.y);
    }

    void onDistanceCorrection(Star& ps, Point& pt, int old_dist, int new_dist) {
        mdiv_.markVertex(pt.x, pt.y);
    };
};



class EquidistMapDivider :  public MapDivider {
    public:
        CompoundMap cmap_;
        
        EquidistMapDivider() :
            MapDivider() 
        { }
        
        void start(lthmi_nav::StartExperiment::Request& req) {
            new (&cmap_) CompoundMap(req.map.info.width, req.map.info.height);
            for (int x=0; x<req.map.info.width; x++)
                for (int y=0; y<req.map.info.height; y++)
                    if (req.map.data[x + y*req.map.info.width]==0)
                        cmap_.setPixel(x,y, FREED); //free
            MapDivider::start(req);
        }
        
        void divide() {
            divideByEquidist();
        }
        
        void divideByEquidist() {
            EquidistMapDividerCWaveProc proc(*this);
            CWave2 cw(cmap_);
            cw.setProcessor(&proc);
            cw.calc(pt_best);
            cmap_.clearDist();
        }
};
}
