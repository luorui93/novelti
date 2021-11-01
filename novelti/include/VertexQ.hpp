#ifndef CWAVE_VERTEXQ_INCLUDE_
#define CWAVE_VERTEXQ_INCLUDE_

//#include <geom/Vertex.hpp>

namespace cwave {
    class VertexQ { //: public Vertex {
    public:
        int x;
        int y;
        VertexQ() : x(0), y(0) {};
        VertexQ(int x0, int y0) : x(x0), y(y0) {};
        bool operator==(const VertexQ& r) const {
            return (x==r.x && y==r.y);
        };
        bool operator!=(const VertexQ& r) const {
            return !(*this==r);
        };
        bool operator>=(const VertexQ& p) const {
            return (y>p.y || (y==p.y && x>=p.x));
        };
    };
    
    using Vertex = VertexQ;
}
#endif
