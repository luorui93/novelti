        /* +------+-------------+
         * |      |             | 
         * |      |      2      |
         * |  0   |-------------|
         * |      |             |
         * |------|      3      |
         * |  1   |             |
         * +------+-------------+  */

#include <lthmi_nav/map_divider.h>

namespace lthmi_nav {

class VertTileMapDivider :  public MapDivider {
    public:
        double half;

        VertTileMapDivider() :
            MapDivider() 
        { }
        
        void divide() {
            measure();
            actuallyDivide();
        }
        
        void measure() {
            half = 0;
            double halfProb = probs_optimal[0] + probs_optimal[1];
            double prob = 0.0;
            double p;
            for (int x=0; x<pdf->info.width;x++)
                for (int y=0; y<pdf->info.height;y++) {
                    //ROS_INFO(">>>>>>>>>>>>>>>>>>>>>>>> %d,%d", x,y);
                    p = pdf->data[y*pdf->info.width+x];
                    if (p>0.0) {
                        prob += p;
                        if (prob > halfProb) {
                            half = x;
                            return;
                        }
                    }
                }
        }
            
        void actuallyDivide() {
            double prob=0;
            int idx, cur_region = 0;
            double p;
            for (int y=0; y<pdf->info.height;y++) {
                for (int x=0; x<=half;x++)  {
                    idx = x + y*(pdf->info.width);
                    p = pdf->data[idx];
                    if (p>=0.0) {
                        map_divided.data[idx] = cur_region;
                        prob += p;
                        if (cur_region==0 && prob >= probs_optimal[cur_region]) {
                            probs_actual[cur_region] = prob;
                            prob = 0.0;
                            cur_region++;
                        }
                    }
                }
            }
            probs_actual[cur_region] = prob;
            prob = 0.0;
            cur_region++;
            
            for (int y=0; y<pdf->info.height;y++) {
                for (int x=half+1; x<pdf->info.width;x++)  {
                    idx = x + y*(pdf->info.width);
                    p = pdf->data[idx];
                    if (p>=0.0) {
                        map_divided.data[idx] = cur_region;
                        prob += p;
                        if (cur_region==2 && prob >= probs_optimal[cur_region]) {
                            probs_actual[cur_region] = prob;
                            prob = 0.0;
                            cur_region++;
                        }
                    }
                }
            }
            probs_actual[cur_region] = prob;
        }
};
}
