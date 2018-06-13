#include <string>
#include <iostream>
#include <vector>
#include <cmath>
#include <stdexcept>

using namespace std;

class InferenceMatrixBad : public std::runtime_error {
public:
    InferenceMatrixBad(const string msg) :
        std::runtime_error(msg) {};
};    

class InferenceMatrix {
public:
    vector<double> infMx_;
    int nCmds_;
    
    InferenceMatrix(vector<double> infMx) :
        infMx_(infMx)
    {
        nCmds_ = (int)floor(sqrt(infMx_.size()));
        if (nCmds_==0 || nCmds_*nCmds_ != infMx_.size())
            throw InferenceMatrixBad("Inference matrix should have length>0, and its length has to be a square of an integer number (number of commands)");
    }
    
    static double calcEntropy(vector<double> probs) {
        double entropy = 0;
        for (const auto& p: probs)
            entropy -= (p==0)? 0: p*log2(p);
        return entropy;
    }
    
    static void normalize(vector<double>& probs) {
        double sum=0;
        for (const auto& p: probs)
            sum+=p;
        for (auto& p: probs)
            p /= sum;
    }
    
    void calcPosteriors(const vector<double>& priors, const int detCmd,
        vector<double>& posteriors //used as output
    ) const {
        double totalProb = 0;
        for (int k=0; k<nCmds_; k++) {
            posteriors[k] = infMx_[detCmd + k*nCmds_] * priors[k]; 
            totalProb += posteriors[k];
        }
        for (auto &p: posteriors)
            p /= totalProb;
    }
    
    void calcUpdateCoefs(const vector<double>& priors, const int detCmd,
        vector<double>& coefs //used as output
    ) const {
        calcPosteriors(priors, detCmd, coefs);
        for (int k=0; k<nCmds_; k++) 
            coefs[k] = priors[k] != 0.0 ? coefs[k]/priors[k] : 0.0;
    }
    
    static string toString(const vector<double>& probs) {
        string s;
        for (const auto& p: probs)
            s += to_string(p)+", ";
        return "["+ s +"]";
    }

    double calcAvgEntropyDecrease(const vector<double>& priors) const {
        double out = calcEntropy(priors);
//         cout << toString(priors) <<"--> " <<out<<"\n";
        vector<double> posteriors(nCmds_);
        for (int k=0; k<nCmds_; k++) {
            calcPosteriors(priors, k, posteriors);
            double postEntropy = calcEntropy(posteriors);
            out -= priors[k]*postEntropy;
//             cout << "    " << toString(posteriors) <<"--> " <<postEntropy << "\n";
            
        }
        return out;
    }
    
    bool nextPriors(vector<double>& priors, double& totalProb, double delta) const {
//             cout << totalProb << endl;
        totalProb += delta;
        for (int k=nCmds_-2;k>=0;k--) {
            if (totalProb<=1) {
                priors[k] += delta;
                break;
            } else {
                if (k==0)
                    return false;
                totalProb -= priors[k];
                priors[k] = 0;
            }
        }
        return true;
    }
    
    double findOptimalPriors(
        vector<double>& optimal_priors, //used as output
        double delta=0.01
    ) const {
        vector<double>& priors = optimal_priors;
        priors = vector<double>(nCmds_,0);
        int id=0, optId;
        double entDec, optEntDec=0, totalProb=0.0;
        do {
            //calc entropy decrease and store if better than before
            priors[nCmds_-1] = 1.0-totalProb;    
//             if (id%100000==0)
//                 cout << toString(priors) << "\n";
            entDec = calcAvgEntropyDecrease(priors);
            if (entDec > optEntDec) {
                optEntDec = entDec;
                optId = id;
            }
//             cout << "    --------------------------------->" << entDec << "\n";            
            id++;
        } while (nextPriors(priors, totalProb, delta));
        
        //get probs from optId
        id=0;
        totalProb=0.0;
        fill(priors.begin(), priors.end(), 0);
        do {
        } while ((id++)!=optId && nextPriors(priors, totalProb, delta));
        priors[nCmds_-1] = 1.0-totalProb;  
        return optEntDec;
    }
    
};


// //Uncomment to test as a compilable executable 

// int main(int argc, char **argv) {
//     InferenceMatrix inf(
//         //{1.0,0.0,     0.5,0.5}
//         //{0.9,0.1,     0.1,0.9}
//         //{0.9,0.05,0.05,     0.05,0.9,0.05,   0.05,0.05,0.9}
//         //{0.91,0.03,0.03,0.03,     0.03,0.9,0.03,0.03,   0.03,0.03,0.9,0.03,   0.03,0.03,0.03,0.9}
//         {   
//             0.90,0.02,0.02,0.02,0.02,     
//             0.02,0.90,0.02,0.02,0.02,   
//             0.02,0.02,0.90,0.02,0.02,
//             0.02,0.02,0.02,0.90,0.02,
//             0.02,0.02,0.02,0.02,0.90
//         }
//     );
//     vector<double> priors;
//     double optEntDec = inf.findOptimalPriors(priors, 0.02);   
//     cout << "Optimal priors: " << inf.toString(priors) <<
//             "\nWeighted entropy decrease for these priors: " << optEntDec <<"\n";
// }
    

