#ifndef CWAVE_ARC_INCLUDE_
#define CWAVE_ARC_INCLUDE_


namespace cwave {
    
    struct RVertex {
        char quad;
        int x;
        int y;
        int eps;
    };
    
    struct Arc {
        RVertex bgn;
        RVertex end;
        int r;
    };
    
} //namespace cwave

#endif
