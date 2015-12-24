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

#ifndef PoseRandomWalker_hpp
#define PoseRandomWalker_hpp

#include <stdio.h>
#include <iostream>
#include <memory>
#include <cmath>
#include <algorithm>

#include "bleloc.h"
#include "Pedometer.hpp"
#include "SystemModel.hpp"
#include "OrientationMeter.hpp"
#include "RandomGenerator.hpp"
#include "Pose.hpp"

namespace loc{
    
    class PoseRandomWalkerProperty{
        Pedometer* pPedomter;
        OrientationMeter* pOrientationMeter;
        
        double mAngularVelocityLimit = 30.0/180.0*M_PI;
        
    public:
        void pedometer(Pedometer* pedometer){
            pPedomter = pedometer;
        }
        void orientationMeter(OrientationMeter* orientationMeter){
            pOrientationMeter = orientationMeter;
        }
        void angularVelocityLimit(double angularVelocityLimit){
            mAngularVelocityLimit = angularVelocityLimit;
        }
        Pedometer* pedometer(){
            return pPedomter;
        }
        OrientationMeter* orientationMeter(){
            return pOrientationMeter;
        }
        double angularVelocityLimit() const{
            return mAngularVelocityLimit;
        }
    };
    
    class PoseRandomWalkerInput{
        long timestamp_;
        long previousTimestamp_;
        
    public:
        void timestamp(long timestamp){
            timestamp_ = timestamp;
        }
        
        void previousTimestamp(long previousTimestamp){
            previousTimestamp_ = previousTimestamp;
        }
        
        long timestamp() const{
            return timestamp_;
        }
        long previousTimestamp() const{
            return previousTimestamp_;
        }
        
    };
    
    class PoseRandomWalker: public SystemModel<State, PoseRandomWalkerInput>{
    private:
        
        RandomGenerator randomGenerator;
        
        PoseProperty poseProperty;
        StateProperty stateProperty;
        PoseRandomWalkerProperty mProperty;
        
        double mVelocityRate = 1.0;
        
    public:
        
        PoseRandomWalker() = default;
        ~PoseRandomWalker() = default;
        
        double velocityRate(){ return mVelocityRate; }
        void velocityRate(double velocityRate) { mVelocityRate = velocityRate; }
        
        PoseRandomWalker& setProperty(PoseRandomWalkerProperty property);
        PoseRandomWalker& setPoseProperty(PoseProperty poseProperty);
        PoseRandomWalker& setStateProperty(StateProperty stateProperty);
        
        std::vector<State> predict(std::vector<State> poses, PoseRandomWalkerInput input) override;
        
        State predict(State state, PoseRandomWalkerInput input) override;
    
    };
    
}

#endif /* PoseRandomWalker_hpp */
