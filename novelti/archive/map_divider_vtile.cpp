        /* +--------------------+
         * |00001111111223333333| 
         * |00001111112223333333|
         * |00011111112223333333|
         * |00011111112233333333|
         * |00011111112233333333|
         * |00011111112233333333|
         * +--------------------+  */

#include <novelti/map_divider.h>

namespace novelti {

class VertTileMapDivider :  public MapDivider {
    public:
        int half;

        VertTileMapDivider() :
            MapDivider() 
        { }
        
        void divide() {
            for (int x=0; x<pdf->info.width;x++)
                for (int y=0; y<pdf->info.height;y++)
                    markVertex(x,y);
        }
};

} //namespace novelti
