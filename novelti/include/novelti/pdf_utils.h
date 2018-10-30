#ifndef NOVELTI_PDF_UTILS_H
#define NOVELTI_PDF_UTILS_H

#include <string>
#include <vector>

namespace novelti {

// Class to keep pdf statistics:
// min, max probs, their indices and total prob
template <class T>
class PdfStats {
public:    
    T   min;
    int min_k;
    T   max;
    int max_k;
    T   total;
    
    PdfStats<T>() :
        min(1.0), min_k(-1), max(0.0), max_k(-1), total(0)
    {}
    
    PdfStats<T>(std::vector<T> pdf) : PdfStats<T>() {
        for (int k=0; k<pdf.size(); k++)
            update(pdf[k],k);
    }
    
    bool update(const T prob, int k) {
        if (prob>=0) {
            total+=prob;
            if (prob>max) { max=prob; max_k = k; }
            if (prob<min) { min=prob; min_k = k; }
            return true;
        }
        return false;
    }
};



namespace PdfUtils {

// Sets all non-negative prob values in pdf to prob_value
template <class T>
void setUniform(std::vector<T>& pdf, const T prob_value) {
    for (auto& p: pdf) //normalize
        if (p>=0)
            p = prob_value;
}


// Sets all values except k-th element to 0
template <class T>
void setDeterministic(std::vector<T>& pdf, int k) {
    for (auto& p: pdf) //normalize
        p = 0;
    pdf[k] = 1.0;
}

// Normalizes pdf (so that it sums up to 1)
template <class T>
void normalize(std::vector<T>& pdf, const T current_total) {
    for (auto& p: pdf) //normalize
        if (p>=0)
            p /= current_total;
}


// Normalizes pdf (so that it sums up to 1)
template <class T>
void normalize(std::vector<T>& pdf) {
    T total = 0.0;
    for (const auto& p: pdf) //normalize
        if (p>=0)
            total += p;
    normalize(pdf, total);
}


// Replaces small probabilities (<eps) in pdf 
// with eps then normalizes pdf
// (yes, it does increase the entropy)
template <class T>
void denullifyNormalize(std::vector<T>& pdf, const T eps) {
    double p, sc=0.0, sa=0.0;
    for (const auto& p: pdf) //replace small probs (<eps) with eps 
        if (p >=0 && p < eps) {
            sc += eps;
            sa += p;
        }
    double c=(1.0-sc)/(1.0-sa);
    for (auto& p: pdf) //normalize
        if (p>=0)
            p = p<eps ? eps : c*p;
}


// Takes large pdf, same size index_maps, calculates accumulative probabilities
// values in pdf with prob<0 are ignored
// inputs: pdf, index_map
// output: accumul_probs
// Also calculates statistics (min, max, total probs, etc)
template <class T>
PdfStats<T> accumulate(
        const std::vector<T>& pdf,          // input
        const std::vector<int>& index_map,  // input
        std::vector<double>& accumul_probs   // output
) {
    PdfStats<T> stats;
    std::fill(accumul_probs.begin(), accumul_probs.end(), 0.0); //accumul_probs = 0-vector
    for (int k=index_map.size()-1; k>=0; k--)
        if (pdf[k]>=0) {
            stats.update(pdf[k],k);
            accumul_probs[index_map[k]] += pdf[k];
        }
    return stats;
}



// Calculates pdf statistics
// (negative probabilities are ignored)
template <class T>
PdfStats<T> calcStats(const std::vector<T>& pdf) {
    PdfStats<T> stats;
    for (int k=pdf.size()-1; k>=0; k--) //find min prob
        stats.update(pdf[k],k);
    return stats;
}


// Updates pdf based on update coefficients and index_map
template <class T>
void update(
        std::vector<T>& pdf,                //output
        const std::vector<int>& index_map,    //input
        const std::vector<double>& coefs         //input
) {
    for (int k=index_map.size()-1; k>0; k--)
        if (pdf[k]>=0)
            pdf[k] *= coefs[index_map[k]];
}

template <class T>
std::string toString(const std::vector<T>& probs) {
    std::string s;
    for (const auto& p: probs)
        s += std::to_string(p)+", ";
    return "["+ s +"]";
}

} //PdfUtils
} //novelti

#endif