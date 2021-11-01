#ifndef CWAVE_ARCWALKER_H_
#define CWAVE_ARCWALKER_H_

namespace cwave {



class QuadPoint {
public:
    char quad; //quadrant
    int x;
    int y;
    int eps; //used for circle drawing eps = x^2+y^2-r^2
    
    QuadPoint(){};
    QuadPoint(char quad0, int x0, int y0, int eps);
    
    void xpp();
    void xmm();
    void ypp();
    void ymm();
    void rpp(int r);
    void rmm(int r);
    void xmm(int& dx);
    void ypp(int& dy);
    bool tooClose(int r, int dx);
    bool tooClose(int r);
    bool tooFar(int r);
    bool areCoordsSame(const QuadPoint& pt);
    bool inQuadrant();
    bool isOdd();
    bool isEvenInclusive();
    bool isOddInclusive();
    void crossZero(int& dx, int& dy);
};



enum TipType {
      //   +---- First move UP (1) or RIGHT (0)
      //   |+--- Double increment=1
      //   ||+-- == Second move is UP (1)
      //   |||
    A = 0b0000,
    B = 0b0010,
    C = 0b0011,
    D = 0b0100,
    E = 0b0110,
    F = 0b0111,
};



class ArcTip {
public: 
    ArcTip() {};
    QuadPoint pt;
    TipType type;
};



static const bool OPEN = 1;
static const bool CLOSED = 0;
static const bool END = 1;
static const bool BGN = 0;

class Boundary {
public:
    Boundary(){};
    Boundary(char quad, int dx, int dy, bool isOpen, bool isEnd);
    Boundary(QuadPoint newhead, int r, bool isOpen, bool isEnd);
    ArcTip calcTip(int radius);
// private:
    bool grow();
    QuadPoint head;
    int dx;
    int dy;
    int leps;  //used for line drawing (line epsilon)
};



static const bool MOVE_LEFT = true;
static const bool MOVE_UP = false;

class ArcWalker {
/* This class iterates through all vertices of an r-ring
 * limited by arc tips
 * (visit(false, *), visitBgn(*), visitEnd(*))
 * for vertices that are close to boundaries of the arc.
 * Does not need to access the map at all.
 */
public:
    ArcWalker(const int radius, const ArcTip& bgnTip, const ArcTip& endTip);
    void run();
    
    bool isFinished();
    void next() ;
    const QuadPoint getPt() {return pt;};
protected:
    const int r;
    QuadPoint pt;

    virtual void bgnInnerVertices() {};
    virtual void endInnerVertices() {};
    virtual void visit(){};
    virtual void visitBgn(const bool isLeftMove){};
    virtual void visitEnd(){};
    virtual void visitZero(){};
    virtual void moveLeft() {};
    virtual void moveUp()   {};
    virtual void moveDiag() {};
private: 
    const ArcTip b;
    ArcTip e;
    int dx;
    int dy;
    
    void preWalk();
    void postWalk();
    void step();
    
    bool loop(const char quad);
    void walk();
    bool endReached();
    bool inQuadrant();
    void updateEndTip();
};

}
#endif
