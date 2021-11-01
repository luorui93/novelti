#ifndef INFERENCE_MATRIX_H
#define INFERENCE_MATRIX_H

#include <string>
#include <vector>
#include <stdexcept>

using namespace std;

class InferenceMatrixBad : public std::runtime_error {
public:
    InferenceMatrixBad(const string msg);
};


class InferenceMatrix {
public:
    const vector<double> infMx_;
    const int nCmds_;
    
    InferenceMatrix(vector<double> infMx);
    static double calcEntropy(const vector<double>& probs);
    static void normalize(vector<double>& probs);
    void calcPosteriors(
        const vector<double>& priors, 
        const int detCmd,
        vector<double>& posteriors //used as output
    ) const;
    
    void calcUpdateCoefs(
        const vector<double>& priors, 
        const int detCmd,
        vector<double>& coefs //used as output
    ) const;
    
    static string toString(const vector<double>& probs);
    double calcAvgEntropyDecrease(const vector<double>& priors) const;
    
    double findOptimalPriors(
        vector<double>& optimal_priors, //used as output
        double delta=0.01
    ) const;
    
protected:    
    bool nextPriors(
        vector<double>& priors, 
        double& totalProb, 
        double delta
    ) const;
};
#endif