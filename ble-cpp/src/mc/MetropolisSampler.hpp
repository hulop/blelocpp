/*******************************************************************************
 * Copyright (c) 2014-2016  IBM Corporation and others
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
*******************************************************************************/

#ifndef MetropolisSampler_hpp
#define MetropolisSampler_hpp

#include <stdio.h>
#include <algorithm>
#include "bleloc.h"
#include "ObservationDependentInitializer.hpp"
#include "ObservationModel.hpp"
#include "StatusInitializer.hpp"
#include "StatusInitializerImpl.hpp"

namespace loc{
    
    typedef enum {
        INIT_WITH_SAMPLE_LOCATIONS,
        INIT_WITH_BEACON_LOCATIONS
    } InitType;
    
    // This class generates samples following p(state|observation) by using the Metropolis algorithm.
    // When withOrdering is set to true, sampling(int n) function returns n largest log-likelihood states from the all generated samples. When withOrdering is false, the latest n samples with the defined interval are returned.
    template<class Tstate, class Tinput>
    class MetropolisSampler : public ObservationDependentInitializer<Tstate, Tinput>{
    public:
        class Parameters{
        public:
            int burnIn = 100;
            int interval = 1;
            double radius2D = 10;
            bool withOrdering = false;
            InitType initType = INIT_WITH_SAMPLE_LOCATIONS;
        };
        bool isVerbose = false;
    private:
        Parameters mParams;
        RandomGenerator randGen;
        Location mStdevLocation;
        std::shared_ptr<ObservationModel<Tstate,Tinput>> mObsModel;
        std::shared_ptr<StatusInitializerImpl> mStatusInitializer;
        Tinput mInput;
        
        void initialize();
        
        Tstate currentState;
        double currentLogLL;
        
        bool isBurnInFinished = false;
        
        std::vector<Tstate> allStates;
        std::vector<double> allLogLLs;
        
        State findInitialMaxLikelihoodState();
        State transitState(Tstate state);
        State transitState(Tstate state, bool transitLoc, bool transitRssiBias);
        
    public:
        
        void parameters(Parameters params);
        void input(const Tinput& input) override;
        void stdevLocation(const Location& stdevLocation){
            mStdevLocation = stdevLocation;
        }
        
        void observationModel(std::shared_ptr<ObservationModel<Tstate, Tinput>> obsModel) override;
        void statusInitializer(std::shared_ptr<StatusInitializerImpl> statusInitializer) override;
        
        void prepare();
        void startBurnIn() override;
        void startBurnIn(int burnIn) override;
        std::vector<Tstate> getAllStates() const override;
        std::vector<double> getAllLogLLs() const override;
        
        bool sample();
        bool sample(bool transitLoc, bool transitRssiBias);
        std::vector<Tstate> sampling(int n) override;
        std::vector<Tstate> sampling(int n, bool withOrdering);
        std::vector<Tstate> sampling(int n, const Location &location) override;
        
        void clear();
        
        void print() const override;
        
    };
    
    
    // Implementation
    template <class Tstate, class Tinput>
    void MetropolisSampler<Tstate, Tinput>::parameters(Parameters params){
        mParams = params;
    }
    
}

#endif /* MetropolisSampler_hpp */
