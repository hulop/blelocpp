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

#ifndef WeakRandomWalker_hpp
#define WeakRandomWalker_hpp

#include <stdio.h>
#include <memory>
#include "SystemModel.hpp"
#include "RandomWalkerMotion.hpp"
#include "PoseRandomWalker.hpp"
#include "PosteriorResampler.hpp"

namespace loc {
    
    class WeakPoseRandomWalkerProperty{
        double pOrientationBiasJump = 0.1; // defines probability of contamination of uniformly-distributed noise to orientation bias
        double pOrientationJump = 0.0; // defines probability of contamination of uniformly-distributed noise to orientation
        double mRandomWalkRate = 0.5; // defines mixture rate between orientation-based prediction and random walk
        
    public:
        using Ptr = std::shared_ptr<WeakPoseRandomWalkerProperty>;
        
        void probabilityOrientationBiasJump(double probabilityOrientationBiasJump_){
            pOrientationBiasJump = probabilityOrientationBiasJump_;
        }
        double probabilityOrientationBiasJump() const{
            return pOrientationBiasJump;
        }
        
        void probabilityOrientationJump(double probabilityOrientationJump_){
            pOrientationJump = probabilityOrientationJump_;
        }
        double probabilityOrientationJump() const{
            return pOrientationJump;
        }
        
        void randomWalkRate(double randomWalkRate_){
            mRandomWalkRate = randomWalkRate_;
        }
        
        double randomWalkRate() const{
            return mRandomWalkRate;
        }
    };
    
    
    // Tentative name
    template<class Ts = State, class Tin = SystemModelInput>
    class WeakPoseRandomWalker: public RandomWalkerMotion<Ts, Tin>{
        using RandomWalkerMotion<Ts, Tin>::currentTimestamp;
        using RandomWalkerMotion<Ts, Tin>::currentYaw;
        using RandomWalkerMotion<Ts, Tin>::wasYawUpdated;
        using RandomWalkerMotion<Ts, Tin>::turningVelocityRate;
        
        PoseProperty::Ptr mPoseProperty;
        StateProperty::Ptr mStateProperty;
        WeakPoseRandomWalkerProperty::Ptr wPRWProperty;
        
        bool enabledPredictions = false;
        bool wasResampled = false;
        long previousTimestampResample = 0;
        
    public:
        using Ptr = std::shared_ptr<WeakPoseRandomWalker<Ts, Tin>>;
        using RandomWalker<Ts, Tin>::predict;
        using RandomWalkerMotion<Ts, Tin>::predict;
        using RandomWalkerMotion<Ts, Tin>::velocityRate;
        using RandomWalkerMotion<Ts, Tin>::setProperty;
        
        WeakPoseRandomWalker() : wPRWProperty(new WeakPoseRandomWalkerProperty) {
            // pass
        }
        
        virtual ~WeakPoseRandomWalker() = default;
        virtual Ts predict(Ts state, Tin input) override;
        virtual void startPredictions(const std::vector<Ts>& states, const Tin&) override;
        virtual void endPredictions(const std::vector<Ts>& states, const Tin&) override;
        virtual void notifyObservationUpdated() override;
        
        virtual void setWeakPoseRandomWalkerProperty(WeakPoseRandomWalkerProperty::Ptr wPRWProperty){
            this->wPRWProperty = wPRWProperty;
        }
        
        void setPoseProperty(PoseProperty::Ptr poseProperty){
            mPoseProperty = poseProperty;
        }
        void setStateProperty(StateProperty::Ptr stateProperty){
            mStateProperty = stateProperty;
        }
        
    };
}

#endif /* PoseRandomWalkerResampling_hpp */
