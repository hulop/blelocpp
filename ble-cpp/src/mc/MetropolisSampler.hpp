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
    
    // This class generates samples following p(state|observation) by using the Metropolis algorithm.
    
    template<class Tstate, class Tinput>
    class MetropolisSampler : public ObservationDependentInitializer<Tstate, Tinput>{
    public:
        class Parameters{
        public:
            int burnIn = 1000;
            int interval = 10;
            double radius2D = 10;
        };
        
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
        
        State findInitialMaxLikelihoodState();
        State transitState(Tstate state);
        
    public:
        
        void parameters(Parameters params);
        void input(const Tinput& input);
        void stdevLocation(const Location& stdevLocation){
            mStdevLocation = stdevLocation;
        }
        
        void observationModel(std::shared_ptr<ObservationModel<Tstate, Tinput>> obsModel);
        void statusInitializer(std::shared_ptr<StatusInitializerImpl> statusInitializer);
        
        void prepare();
        void startBurnIn();
        void startBurnIn(int burnIn);
        bool sample();
        std::vector<Tstate> sampling(int n);
    };
    
    
    // Implementation
    template <class Tstate, class Tinput>
    void MetropolisSampler<Tstate, Tinput>::parameters(Parameters params){
        mParams = params;
    }
    
}

#endif /* MetropolisSampler_hpp */
