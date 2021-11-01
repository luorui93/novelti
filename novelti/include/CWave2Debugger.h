#ifndef CWAVE2DEBUGGER_H_
#define CWAVE2DEBUGGER_H_

namespace cwave {
    #ifndef CWAVE2_DBG
        #define CWAVE2_DBG(cwave, action, ...) (cwave).debugger.action((cwave), ##__VA_ARGS__)
    #endif
    class CWave2;
    class Point;
    class Star;
    class Beam;
    class Walker;
    class Boundary;
    class CWave2Debugger {
        public:
            virtual void onBeamGrow(CWave2& cw, Star& s, Beam& bm)=0;
            virtual void onStarAdded(CWave2& cw, Star& s)=0;
            virtual void onVisitPair(CWave2& cw, Walker& w, bool reg, bool nbp)=0;
            virtual void onCheckPixel(CWave2& cw, Point& p)=0;
            /*virtual void onSetPointDist(CWave2& cw, Point& p, int val, bool is_nbp)=0;*/
            virtual void onSetPointDist(CWave2& cw, Point& p, int val, bool is_nbp, bool overlap, bool to_mark)=0;
            virtual void print(CWave2& cw)=0;
            virtual void onBoundaryAddPixel(CWave2& cw, Star& s, Boundary& b)=0;
            virtual void onStarRemoved(CWave2& cw, Star& s)=0;
            virtual void onTipUpdate(CWave2& cw, Star& s, Boundary& b)=0;
    };

}
#endif
