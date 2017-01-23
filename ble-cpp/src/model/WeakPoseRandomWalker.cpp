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

#include "WeakPoseRandomWalker.hpp"

namespace loc{
    
    template<class Ts, class Tin>
    Ts WeakPoseRandomWalker<Ts, Tin>::predict(Ts state, Tin input){
        auto& mRandGen = RandomWalker<Ts, Tin>::mRandGen;
        auto& mRWMotionProperty = RandomWalkerMotion<Ts,Tin>::mRWMotionProperty;
        const auto& mPedometer = mRWMotionProperty->pedometer();
        const auto& mOrientationMeter = mRWMotionProperty->orientationMeter();
        
        long t_pre = input.previousTimestamp();
        long t_cur = input.timestamp();
        double dt = (t_cur-t_pre) * input.timeUnit();
        double sqdt = std::sqrt(dt);
        
        if(dt < 0){
            std::cerr << "Inconsistent timestamp found in WeakPoseRandomWalker. Location was not updated." << std::endl;
            return state;
        }
        
        if(mPedometer && mOrientationMeter){
            double nSteps = mPedometer->getNSteps();
            double movLevel = RandomWalkerMotion<Ts,Tin>::movingLevel();
            double yaw = mOrientationMeter->getYaw();
            
            if(dt<input.timeUnit()){
                throw std::runtime_error("Time increment is too small in WeakPoseRandomWalker.");
            }
            // Compute velocity rate to reduce velocity when turning
            if(mRWMotionProperty->usesAngularVelocityLimit()){
                if(!wasYawUpdated){ // for the initial loop
                    currentTimestamp = t_cur;
                    currentYaw = yaw;
                    wasYawUpdated = true;
                }
                if(currentTimestamp!=t_cur){
                    currentTimestamp = t_cur;
                    double previousYaw = currentYaw;
                    currentYaw = yaw;
                    if(previousYaw<-M_PI || M_PI<previousYaw){
                        BOOST_THROW_EXCEPTION(LocException("previous yaw is out of range."));
                    }
                    if(currentYaw<-M_PI || M_PI<currentYaw){
                        BOOST_THROW_EXCEPTION(LocException("current yaw is out of range."));
                    }
                    double oriDiff = Pose::computeOrientationDifference(previousYaw, currentYaw);
                    double angularVelocity = oriDiff/dt;
                    double angularVelocityLimit = mRWMotionProperty->angularVelocityLimit();
                    turningVelocityRate = std::sqrt(1.0 - std::min(1.0, std::pow(angularVelocity/angularVelocityLimit,2)));
                }
            }else{
                turningVelocityRate = 1.0;
            }
            
            // Compute sigma for RandomWalkerMotion
            double sigma = movLevel>0 ? mRWMotionProperty->sigmaMove : mRWMotionProperty->sigmaStop;
            // Multiply sigma by velocity rate and turning velocity rate
            sigma = sigma * velocityRate() * turningVelocityRate;
            
            // Add noise to (actually) static parameters just after resampling
            if(wasFiltered){
                // Update only if a device is moving by active moving (not passive moveing)
                if(nSteps > 0){
                    double dt_longTmp = (t_cur - previousTimestampResample) * input.timeUnit(); //time interval between resampling steps
                    double dt_long; // cap dt_long value
                    if(wPRWProperty->maxLongTimestep() < dt_longTmp){
                        dt_long = wPRWProperty->maxLongTimestep();
                    }else{
                        dt_long = dt_longTmp;
                    }
                    
                    double sqdt_long = std::sqrt(dt_long);
                    // Perturb variables in State (orientationBias, rssiBias)
                    double oriTmp;
                    if( mRandGen->nextDouble() < wPRWProperty->probabilityOrientationBiasJump()){
                        oriTmp = Pose::normalizeOrientaion( 2.0 * M_PI * (mRandGen->nextDouble()-0.5));
                    }else{
                        oriTmp = mRandGen->nextWrappedNormal(state.orientationBias(),
                                                             mStateProperty->diffusionOrientationBias() * sqdt_long );
                    }
                    state.orientationBias(oriTmp);
                    state.rssiBias(mRandGen->nextTruncatedGaussian(state.rssiBias(), mStateProperty->diffusionRssiBias() * sqdt_long , mStateProperty->minRssiBias(), mStateProperty->maxRssiBias()));
                    
                    // Perturb variables in Pose (normal velocity)
                    double nV = state.normalVelocity();
                    nV = mRandGen->nextTruncatedGaussian(state.normalVelocity(),
                                                         mPoseProperty->diffusionVelocity() * sqdt_long,
                                                         mPoseProperty->minVelocity(),
                                                         mPoseProperty->maxVelocity());
                    state.normalVelocity(nV);
                    
                    // Assign orientationAlignment
                    state.orientationAlignment(0.0);
                    if( mRandGen->nextDouble() < wPRWProperty->probabilityBackwardMove()){
                        double oriBW = M_PI;
                        state.orientationAlignment(oriBW);
                    }
                }
            }
            
            // update orientation
            double orientationActual = yaw - state.orientationBias();
            if(movLevel>0 ){
                // Add noise to orientation
                orientationActual = mRandGen->nextWrappedNormal(orientationActual, mPoseProperty->stdOrientation() * sqdt);
                if( mRandGen->nextDouble() < wPRWProperty->probabilityOrientationJump() ){
                    orientationActual = Pose::normalizeOrientaion( 2.0 * M_PI * (mRandGen->nextDouble() - 0.5));
                }
            }
            state.orientation(orientationActual);
            
            // Update velocity
            double v = 0.0;
            if(nSteps > 0){
                double nV = state.normalVelocity();
                v = nV * velocityRate() * turningVelocityRate;
            }
            if(relativeVelocity() > 0){
                v += mRandGen->nextTruncatedGaussian(relativeVelocity(),
                                                     mPoseProperty->diffusionVelocity()*sqdt,
                                                     mPoseProperty->minVelocity(),
                                                     mPoseProperty->maxVelocity());
            }
            state.velocity(v);
            
            // consider orientation alignment
            double oriActAl = Pose::normalizeOrientaion(orientationActual + state.orientationAlignment());
            
            // Update in-plane coordinate.
            double dx_v = state.velocity()*std::cos(oriActAl) * dt;
            double dy_v = state.velocity()*std::sin(oriActAl) * dt;
            
            double dx_noise = sigma * mRandGen->nextGaussian() * sqdt;
            double dy_noise = sigma * mRandGen->nextGaussian() * sqdt;
            
            double poseRwr = wPRWProperty->poseRandomWalkRate();
            double rwr = wPRWProperty->randomWalkRate();
            double x = state.x() + poseRwr * dx_v + rwr * dx_noise;
            double y = state.y() + poseRwr * dy_v + rwr * dy_noise;
            
            State statePred(state);
            statePred.x(x);
            statePred.y(y);
            
            return statePred;
        }else{
            if(!mPedometer){
                BOOST_THROW_EXCEPTION(LocException("Pedometer is not set to WeakPoseRandomWalker."));
            }
            else{
                BOOST_THROW_EXCEPTION(LocException("OrientationMeter is not set to WeakPoseRandomWalker."));
            }
        }
    }
    
    template<class Ts, class Tin>
    void WeakPoseRandomWalker<Ts, Tin>::startPredictions(const std::vector<Ts>& states, const Tin& input){
        enabledPredictions = true;
        if(previousTimestampResample==0){
            previousTimestampResample = input.timestamp();
        }
        // pass
    }
    
    template<class Ts, class Tin>
    void WeakPoseRandomWalker<Ts, Tin>::endPredictions(const std::vector<Ts>& states, const Tin& input){
        if(wasFiltered){
            previousTimestampResample = input.timestamp();
        }
        wasFiltered = false;
    }
    
    
    template<class Ts, class Tin>
    void WeakPoseRandomWalker<Ts, Tin>::notifyObservationUpdated(){
        wasFiltered = true;
    }
    
    template class WeakPoseRandomWalker<State, SystemModelInput>;
    
}
