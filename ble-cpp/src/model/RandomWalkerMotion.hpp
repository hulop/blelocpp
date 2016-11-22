/*******************************************************************************
 * Copyright (c) 2014, 2016  IBM Corporation and others
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

#ifndef RandomWalkerMotion_hpp
#define RandomWalkerMotion_hpp

#include <stdio.h>
#include "RandomWalker.hpp"
#include "Pedometer.hpp"
#include "OrientationMeter.hpp"

namespace loc{
    
    class RandomWalkerMotionProperty{
    public:
        using Ptr = std::shared_ptr<RandomWalkerMotionProperty>;
        double sigmaStop = 0.05;
        double sigmaMove = 1.0;
        
        RandomWalkerMotionProperty& pedometer(Pedometer::Ptr ped){
            mPedometer = ped;
            return *this;
        };
        Pedometer::Ptr pedometer() const{
            return mPedometer;
        }
        RandomWalkerMotionProperty& orientationMeter(OrientationMeter::Ptr ori){
            mOrientationMeter = ori;
            return *this;
        };
        OrientationMeter::Ptr orientationMeter() const{
            return mOrientationMeter;
        }

        RandomWalkerMotionProperty& usesAngularVelocityLimit(bool usesAngularVelocityLimit){
            mUsesAngularVelocityLimit = usesAngularVelocityLimit;
            return *this;
        };
        bool usesAngularVelocityLimit() const{
            return mUsesAngularVelocityLimit;
        }
        RandomWalkerMotionProperty& angularVelocityLimit(double angularVelocityLimit){
            mAngularVelocityLimit = angularVelocityLimit;
            return *this;
        };
        double angularVelocityLimit() const{
            return mAngularVelocityLimit;
        }
        
    private:
        Pedometer::Ptr mPedometer;
        OrientationMeter::Ptr mOrientationMeter;

        bool mUsesAngularVelocityLimit = false;
        double mAngularVelocityLimit = 30.0/180.0*M_PI;
    };
        
    template<class Ts, class Tin>
    class RandomWalkerMotion: public RandomWalker<Ts, Tin>, public SystemModelVelocityAdjustable, public SystemModelMovementControllable{
    public:
        RandomWalkerMotion(){
            mRWMotionProperty.reset(new RandomWalkerMotionProperty);
        };
        virtual ~RandomWalkerMotion() = default;
        
        using Ptr = std::shared_ptr<RandomWalkerMotion>;
        
        virtual Ts predict(Ts state, Tin input) override;
        virtual RandomWalkerMotion& setProperty(RandomWalkerMotionProperty::Ptr);

    protected:
        RandomWalkerMotionProperty::Ptr mRWMotionProperty;
        double turningVelocityRate = 1.0;
        long currentTimestamp;
        double currentYaw;
        bool wasYawUpdated = false;
        
        virtual double movingLevel();
    };
}

#endif /* RandomWalkerMotion_hpp */
