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
        bool mDoesUpdateWhenStopping = false;
        
    public:
        using Ptr = std::shared_ptr<PoseRandomWalkerProperty>;
        
        void pedometer(Pedometer* pedometer){
            pPedomter = pedometer;
        }
        void orientationMeter(OrientationMeter* orientationMeter){
            pOrientationMeter = orientationMeter;
        }
        void angularVelocityLimit(double angularVelocityLimit){
            mAngularVelocityLimit = angularVelocityLimit;
        }
        void doesUpdateWhenStopping(bool doesUpdate){
            mDoesUpdateWhenStopping = doesUpdate;
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
        bool doesUpdateWhenStopping() const{
            return mDoesUpdateWhenStopping;
        }
    };
    
    class PoseRandomWalker: public SystemModel<State, SystemModelInput>, public SystemModelVelocityAdjustable, public SystemModelMovementControllable{

    protected:        
        RandomGenerator randomGenerator;
        PoseProperty::Ptr poseProperty = PoseProperty::Ptr(new PoseProperty);
        StateProperty::Ptr stateProperty = StateProperty::Ptr(new StateProperty);
        PoseRandomWalkerProperty::Ptr mProperty = PoseRandomWalkerProperty::Ptr(new PoseRandomWalkerProperty);
        
    public:
        
        PoseRandomWalker() = default;
        virtual ~PoseRandomWalker() = default;
                
        virtual PoseRandomWalker& setProperty(PoseRandomWalkerProperty::Ptr property);
        virtual PoseRandomWalker& setPoseProperty(PoseProperty::Ptr poseProperty);
        virtual PoseRandomWalker& setStateProperty(StateProperty::Ptr stateProperty);
        
        virtual std::vector<State> predict(std::vector<State> poses, SystemModelInput input) override;
        virtual State predict(State state, SystemModelInput input) override;
        
        virtual double movingLevel();
    };
    
}

#endif /* PoseRandomWalker_hpp */
