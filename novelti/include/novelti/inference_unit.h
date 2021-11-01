#ifndef INFERENCE_UNIT_H
#define INFERENCE_UNIT_H

#include <vector>
#include <novelti/StartExperiment.h>

namespace novelti {

class InferenceUnit {
public:
    /*
     * Initialize class members based on req, register publishers and subscribers
     */
    virtual void start(StartExperiment::Request& req) = 0;
    
    /*
     * Unregister publishers
     */
    virtual void stop() = 0; 
    
    /*
     * Start/restart inference
     *  output:  new_priors
     */
    virtual void initPriors(std::vector<double>& new_priors) = 0;
    
    /*
     * Updates priors to posteriors
     *  input:   coefs
     *  input:   cmd
     *  output:  new_priors
     */
    virtual void update(
        const std::vector<double>& coefs, 
        const int cmd, 
        std::vector<double>& new_priors
    ) = 0;
    
    /*
     * Executed when an option is inferred
     *  input: inferredCmd
     */
    virtual void onInferred(int inferredCmd) = 0;
    
    /*
     * Executed after new priors (posteriors) are calculated
     */
    virtual void act() = 0;
};    

}

#endif