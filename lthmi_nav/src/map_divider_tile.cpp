 /*

Vert tile:
 +--------------------+
 |00001111111223333333| 
 |00001111112223333333|
 |00011111112223333333|
 |00011111112233333333|
 |00011111112233333333|
 |00011111112233333333|
 +--------------------+  

Horix tile:
 +--------------------+
 |00000000000000000000|
 |00001111111111111111|
 |11111111111111111111|
 |11111111112222222222|
 |22222222222222222222|
 |22222222222222222233|
 +--------------------+  */

#include <lthmi_nav/map_divider.h>

namespace lthmi_nav {

class TileMapDivider :  public MapDivider {
    public:
        enum TileType {HORIZ, VERT};
        
        TileType type_;
        
        TileMapDivider(TileType type) :
            MapDivider(),
            type_(type)
        { }
        
        void divide() {
            switch(type_) {
                case HORIZ: 
                    divideIntoHorizTiles(); break;
                default:
                    divideIntoVertTiles(); break;
            }
        }
        
        void divideIntoHorizTiles() {
            for (int y=0; y<pdf->info.height;y++)
                for (int x=0; x<pdf->info.width;x++)
                    markVertex(x,y);
        }
        void divideIntoVertTiles() {
            for (int x=0; x<pdf->info.width;x++)
                for (int y=0; y<pdf->info.height;y++)
                    markVertex(x,y);
        }
};

} //namespace lthmi_nav
