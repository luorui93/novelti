#ifndef MAP_IF_H_
#define MAP_IF_H_


using namespace std;

namespace lthmi_nav {
    
    template <class CellType>
    class MapIf {
        public:
            //virtual MapIf(int w, int h, CellType init_val)  = 0;
            virtual ~MapIf() {};
            
            virtual CellType get(int x, int y) = 0;              __attribute__((always_inline));
            virtual void set(int x, int y, CellType val) = 0;    __attribute__((always_inline));
            virtual int width() = 0;                            __attribute__((always_inline));
            virtual int height() = 0;                            __attribute__((always_inline));
            virtual void clean_dist()=0;
            virtual void resize(int w, int h, CellType val) = 0;
    };
    
    
}
#endif
