#ifndef CWAVE_ARCVERTICES_INCLUDE_
#define CWAVE_ARCVERTICES_INCLUDE_

#include <memory>
#include <iterator>
#include <VertexQ.hpp>
#include <Arc.hpp>

namespace cwave {
    
    
    class ArcVertices {
        private:
            class Impl;
            std::unique_ptr<Impl> pimpl_;
        public:
            class iterator {
                using iterator_category = std::input_iterator_tag;
                private:
                    Impl* pimpl_;
                public:
                    iterator(Impl* pimpl);
                    const iterator& operator++();
                    Vertex operator*() const;
                    bool operator!=(const iterator& it);
            };
            ArcVertices(const Arc& arc, const Vertex& center={0,0});
            ~ArcVertices();
            iterator begin();
            iterator end()   const;
    };
    
} //namespace cwave

#endif
