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

#include "PedometerWalkingState.hpp"

namespace loc{
    
    PedometerWalkingState::PedometerWalkingState(PedometerWalkingStateParameters parameters){
        mParameters = parameters;
    }
    

    PedometerWalkingState& PedometerWalkingState::putAcceleration(Acceleration acceleration){
        
        long timestamp = acceleration.timestamp();
        if(prevUpdateTime==0){
            prevUpdateTime = timestamp;
        }
        
        if(timestamp < prevUpdateTime){
            std::cout << "Inconsistent timestamp was found in Pedometer. Pedometer status was resetted." << std::endl;
            this->reset();
        }
        
        double ax = acceleration.ax();
        double ay = acceleration.ay();
        double az = acceleration.az();
        
        double amp = G*std::sqrt(ax*ax + ay*ay + az*az);
        double diffAmp = amp - G;
        
        amplitudesQueue.push_back(diffAmp);
        if(amplitudesQueue.size()>queue_limit){
            amplitudesQueue.pop_front();
        }
        
        if(amplitudesQueue.size()< queue_min){
            nSteps = 0.0;
        }else{
            size_t n = amplitudesQueue.size();
            double mean = 0;
            double variance = 0;
            
            for(size_t i=0; i<n ; i++){
                mean += amplitudesQueue[i];
            }
            mean/=n;
            for(size_t i=0; i<n ; i++){
                double diff = (amplitudesQueue[i] - mean);
                variance += diff*diff;
            }
            variance /= n;
            double std = sqrt(variance);
            
            if(std > mParameters.walkDetectSigmaThreshold()){
                nSteps = nStepsConst;
            }else{
                nSteps = 0.0;
            }
        }
        
        double dTime = (timestamp-prevUpdateTime)/1000.0;
        if( dTime >= mParameters.updatePeriod()){
            prevUpdateTime = timestamp;
            isUpdated_ = true;
        }else{
            isUpdated_ = false;
        }
        
        return *this;
    }
    
    bool PedometerWalkingState::isUpdated(){
        return isUpdated_;
    }
    
    
    double PedometerWalkingState::getNSteps(){
        return nSteps;
    }
    
    void PedometerWalkingState::reset(){
        prevUpdateTime = 0;
        isUpdated_ = false;
        amplitudesQueue.clear();
    }
    
    
}
