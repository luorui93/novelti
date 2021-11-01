#include <ArcWalker.hpp>
#include <math.h>

namespace cwave {

    class SectorQuadPoints {
        public:
            class iterator {
                private:
                    Boundary bgnBry;
                    Boundary endBry;
                    int maxRadius;
                    int r;
                protected:
                    QuadPoint pt;
                public:
                    ArcWalker* walker;
                    iterator();
                    ~iterator();
                    iterator(Boundary bgn, Boundary end, int radius);
                    const iterator& operator++();
                    QuadPoint operator*() const;
                    bool operator!=(const iterator& it);
            };
            SectorQuadPoints(double startAngle, double stopAngle, int radius);
            SectorQuadPoints(double startAngle, double stopAngle, int r, int outerRadius);
            iterator begin() const;
            iterator end()   const { return sentinel; };
            static Boundary calcBoundary(int x, int y, bool isEnd);
        protected:
            int r;
            Boundary bgnBry;
            Boundary endBry;
            static const iterator sentinel; //end iterator
    };
    
    class Point {
    public:    
        int x;
        int y;
        Point(int x0, int y0) : x(x0), y(y0) {};
    };
    
    class SectorPoints : public SectorQuadPoints {
        public:
            class iterator : public SectorQuadPoints::iterator {
                private:
                    Point* cp;
                public:
                    iterator(Point* centerPtr, Boundary bgn, Boundary end, int radius);
                    iterator();
                    Point operator*() const;
            };
            SectorPoints(Point center, double startAngle, double stopAngle, int radius) : 
                SectorPoints(center, startAngle, stopAngle, radius, radius) {};
            SectorPoints(Point center, double startAngle, double stopAngle, int r, int outerRadius);
            iterator begin();
            iterator end()   const { return sentinel; };
        private:
            Point c;
            static const iterator sentinel; //end iterator
    };
    
    
}//namespace cwave