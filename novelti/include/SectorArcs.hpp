#ifndef CWAVE_SECTORARCS_INCLUDE_
#define CWAVE_SECTORARCS_INCLUDE_

#include <Arc.hpp>
#include <iterator>
#include <memory> //for unique_ptr

namespace cwave {
    
    class SectorArcs {
        private:
            class Impl;
            std::unique_ptr<Impl> pimpl_;
        public:
            class iterator {
                using iterator_category = std::input_iterator_tag;
                private:
                    Impl* pimpl_;
                public:
                    iterator(Impl* arcs);
                    const iterator& operator++();
                    Arc operator*() const;
                    bool operator!=(const iterator& it);
            };
            SectorArcs(double startAngle, double stopAngle, int radius, int outerRadius=1000);
            ~SectorArcs();
            iterator begin();
            iterator end()   const;
            
    };
    
    
} //namespace cwave

#endif