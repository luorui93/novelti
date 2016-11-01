        /* +------+-------------+
         * |      |             | 
         * |      |      1      |
         * |  0   |-------------|
         * |      |      2      |
         * |------|-------------|
         * |  1   |      3      |
         * +------+-------------+  */

#include <lthmi_nav/map_divider.h>

namespace lthmi_nav {

class VertChessMapDivider :  public MapDivider {
    public:
        int half;

        VertChessMapDivider() :
            MapDivider() 
        { }
        
        void divide() {
            measure();
            actuallyDivide();
        }
        void measure() {
            half = 0;
            double halfProb = probs_optimal[0] + probs_optimal[1];
            double prob=0.0;
            for (int x=0; x<pdf->info.width;x++)
                for (int y=0; y<pdf->info.height;y++)
                    if (pdf->data[y*pdf->info.width+x]>0.0) {
                        prob += pdf->data[y*pdf->info.width+x];
                        if (prob > halfProb) {
                            half = x;
                            //ROS_INFO(">>>>>>>>>>>>>>>>>>>>>>>> x_half=%d, halfProb=%f", half, halfProb);
                            return;
                        }
                    }
        }
            
        void actuallyDivide() {
            for (int y=0; y<pdf->info.height;y++)
                for (int x=0; x<=half;x++) 
                    markVertex(x,y);
           
            for (int y=0; y<pdf->info.height;y++)
                for (int x=half+1; x<pdf->info.width;x++)
                    markVertex(x,y);
        }
};
}