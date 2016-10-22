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
        int half;

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
//         void measure() {
//             half = 0;
//             double halfProb = probs_optimal[0] + probs_optimal[1];
//             double colprob, xprob=0.0;
//             for (int x=0; x<pdf->info.width;x++) {
//                 colprob = 0.0;
//                 for (int y=0; y<pdf->info.height;y++)
//                     if (pdf->data[y*pdf->info.width+x]>0.0)
//                         colprob += pdf->data[y*pdf->info.width+x];
//                 xprob += colprob;
//                 if (xprob > halfProb) {
//                     half = x;
//                     if (xprob != colprob)
//                         half--;
//                    ROS_INFO(">>>>>>>>>>>>>>>>>>>>>>>> x_half=%d, halfProb=%f", half, halfProb);
//                    return;
//                 }
//             }
//         }
//             
//         void actuallyDivide() {
//             double prob=0;
//             int idx, cur_region = 0;
//             double p;
//             for (int y=0; y<pdf->info.height;y++) {
//                 for (int x=0; x<=half;x++)  {
//                     idx = x + y*(pdf->info.width);
//                     p = pdf->data[idx];
//                     if (p>=0.0) {
//                         prob += p;
//                         if (cur_region==0 && prob > probs_optimal[cur_region] && prob != p) {
//                             probs_actual[cur_region] = prob-p;
//                             prob = p;
//                             cur_region++;
//                         }
//                         map_divided.data[idx] = cur_region;
//                     }
//                 }
//             }
//             probs_actual[cur_region] = prob;
//             prob = 0.0;
//             cur_region++;
//             
//             for (int y=0; y<pdf->info.height;y++) {
//                 for (int x=half+1; x<pdf->info.width;x++)  {
//                     idx = x + y*(pdf->info.width);
//                     p = pdf->data[idx];
//                     if (p>=0.0) {
//                         prob += p;
//                         if (cur_region==2 && prob >= probs_optimal[cur_region] && prob != p) {
//                             probs_actual[cur_region] = prob-p;
//                             prob = p;
//                             cur_region++;
//                         }
//                         map_divided.data[idx] = cur_region;
//                     }
//                 }
//             }
//             probs_actual[cur_region] = prob;
//         }
};
}
