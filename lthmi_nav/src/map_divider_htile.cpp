        /* +--------------------+
         * |00000000000000000000|
         * |00001111111111111111|
         * |11111111111111111111|
         * |11111111112222222222|
         * |22222222222222222222|
         * |22222222222222222233|
         * +--------------------+  */

#include <lthmi_nav/map_divider.h>

namespace lthmi_nav {

class HorizTileMapDivider :  public MapDivider {
    public:
        int half;

        HorizTileMapDivider() :
            MapDivider() 
        { }
        
        void divide() {
            for (int y=0; y<pdf->info.height;y++)
                for (int x=0; x<pdf->info.width;x++)
                    markVertex(x,y);
        }
};

} //namespace lthmi_nav
