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

#include "MetropolisAlgorithm.hpp"

namespace loc{
    
    // Implementation
    template <class Tstate, class Tinput>
    void MetropolisSampler<Tstate, Tinput>::burnIn(int burnIn){
        mBurnIn = burnIn;
    }
    
    template <class Tstate, class Tinput>
    void MetropolisSampler<Tstate, Tinput>::radius2D(double radius2D){
        mRadius2D = radius2D;
    }
    
    template <class Tstate, class Tinput>
    void MetropolisSampler<Tstate, Tinput>::input(const Tinput& input){
        mInput = input;
    }
    
    template <class Tstate, class Tinput>
    void MetropolisSampler<Tstate, Tinput>::observationModel(std::shared_ptr<ObservationModel<Tstate, Tinput>> obsModel){
        mObsModel = obsModel;
    }
    
    template <class Tstate, class Tinput>
    void MetropolisSampler<Tstate, Tinput>::statusInitializer(std::shared_ptr<StatusInitializerImpl> statusInitializer){
        mStatusInitializer = statusInitializer;
    }
    
    template <class Tstate, class Tinput>
    State MetropolisSampler<Tstate, Tinput>::findInitialMaxLikelihoodState(){
        auto locations = mStatusInitializer->extractLocationsCloseToBeacons(mInput, mRadius2D);
        auto states = mStatusInitializer->initializeStatesFromLocations(locations);
        std::vector<double> logLLs = mObsModel->computeLogLikelihood(states, mInput);
        auto iterMax = std::max_element(logLLs.begin(), logLLs.end());
        size_t index = std::distance(logLLs.begin(), iterMax);
        Tstate locMaxLL = states.at(index);
        return locMaxLL;
    }
    
    template <class Tstate, class Tinput>
    std::vector<State> MetropolisSampler<Tstate, Tinput>::sampling(int n, Tstate initialState){
        
        std::vector<Tstate> sampledStates;
        int i=0;
        Tstate state = initialState;
        double logLL;
        {
            std::vector<Tstate> ss = {state};
            logLL = mObsModel->computeLogLikelihood(ss, mInput).at(0);
        }
        
        int count = 0;
        int countAccepted = 0;
        while(true){
            Tstate stateNew = transitState(state);
            double logLLNew;
            {
                std::vector<Tstate> ss = {stateNew};
                logLLNew = mObsModel->computeLogLikelihood(ss, mInput).at(0);
            }
            double r = std::exp(logLLNew-logLL); // p(xnew)/p(x) = exp(log(p(xnew))-log(p(xpre)))
            
            bool isAccepted = false;
            if(randGen.nextDouble() < r){
                state = stateNew;
                isAccepted = true;
            }
            i++;
            
            if(i>mBurnIn){
                count++;
                if(isAccepted){
                    countAccepted++;
                }
                if(i%mInterval==0){
                    sampledStates.push_back(state);
                }
                if(sampledStates.size()>=n){
                    break;
                }
            }
        }
        
        std::cout << "M-H acceptance rate = " << (double)countAccepted / (double) count << std::endl;
        
        return sampledStates;
    }
    
    template <class Tstate, class Tinput>
    std::vector<Tstate> MetropolisSampler<Tstate, Tinput>::sampling(int n){
        Tstate locMaxLL = findInitialMaxLikelihoodState();
        return sampling(n, locMaxLL);
    }
    
    template <class Tstate, class Tinput>
    State MetropolisSampler<Tstate, Tinput>::transitState(Tstate state){
        // update variables that affect mainly likelihood
        Tstate stateNew(state);
        auto locNew = mStatusInitializer->perturbLocation(stateNew);
        stateNew.x(locNew.x());
        stateNew.y(locNew.y());
        return stateNew;
    }
    
    // explicit instantiation
    template class MetropolisSampler<State, Beacons>;
    
}