/*******************************************************************************
 * Copyright (c) 2014, 2015  IBM Corporation and others
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

#ifndef PedometerWalkingState_hpp
#define PedometerWalkingState_hpp

#include <stdio.h>
#include <deque>
#include "Pedometer.hpp"

namespace loc{
    
    class PedometerWalkingStateParameters{
        double walkDetectStdevWindow_ = 0.8;
        double walkDetectSigmaThreshold_ = 0.6;
        
        double updatePeriod_ = 0.1; // second
        
    public:
        
        PedometerWalkingStateParameters& walkDetectStdevWindow(double walkDetectStdevWindow){
            walkDetectStdevWindow_ = walkDetectStdevWindow;
            return *this;
        }
        
        PedometerWalkingStateParameters& walkDetectSigmaThreshold(double walkDetectSigmaThreshold){
            walkDetectSigmaThreshold_ = walkDetectSigmaThreshold;
            return *this;
        }
        
        PedometerWalkingStateParameters& updatePeriod(double updatePeriod){
            updatePeriod_ = updatePeriod;
            return *this;
        }
        
        double walkDetectStdevWindow() const{
            return walkDetectStdevWindow_;
        }
        
        double walkDetectSigmaThreshold() const{
            return walkDetectSigmaThreshold_;
        }
        
        double updatePeriod() const{
            return updatePeriod_;
        }
        
    };
    
    
    class PedometerWalkingState : public Pedometer{
        
    private:
        long prevUpdateTime = 0;
        bool isUpdated_=false;
        double nSteps = 0;
        
        std::deque<double> amplitudesQueue;
        size_t queue_limit = 80;
        size_t queue_min = 80;
        
        double nStepsConst = 0.1;
        
        std::mutex mtx;
        double G = 9.8;
        
        PedometerWalkingStateParameters mParameters;
        
    public:
        PedometerWalkingState(){}
        PedometerWalkingState(PedometerWalkingStateParameters parameters);
        ~PedometerWalkingState(){}
        
        PedometerWalkingState& putAcceleration(Acceleration acceleration) override;
        bool isUpdated() override;
        double getNSteps() override;
        void reset() override;
    };
    
}

#endif /* PedometerWalkingState_hpp */
