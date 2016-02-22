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

#ifndef MetropolisAlgorithm_hpp
#define MetropolisAlgorithm_hpp

#include <stdio.h>
#include <algorithm>
#include "bleloc.h"
#include "ObservationDependentInitializer.hpp"
#include "ObservationModel.hpp"
#include "StatusInitializer.hpp"
#include "StatusInitializerImpl.hpp"

namespace loc{
    
    template<class Tstate, class Tinput>
    class MetropolisSampler : public ObservationDependentInitializer<Tstate, Tinput>{
    private:
        int mBurnIn = 5000;
        int mInterval = 10;
        double mRadius2D = 10; // [m]
        RandomGenerator randGen;
        std::shared_ptr<ObservationModel<Tstate,Tinput>> mObsModel;
        std::shared_ptr<StatusInitializerImpl> mStatusInitializer;
        Tinput mInput;
        
        State findInitialMaxLikelihoodState();
        std::vector<State> sampling(int n, Tstate initialState);
        State transitState(Tstate state);
        
    public:
        void burnIn(int burnIn);
        void radius2D(double radius2D);
        void input(const Tinput& input);
        
        void observationModel(std::shared_ptr<ObservationModel<Tstate, Tinput>> obsModel);
        void statusInitializer(std::shared_ptr<StatusInitializerImpl> statusInitializer);
        
        std::vector<Tstate> sampling(int n);
    };
    
}

#endif /* MetropolisAlgorithm_hpp */
