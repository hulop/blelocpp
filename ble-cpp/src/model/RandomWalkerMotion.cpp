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

#include "RandomWalkerMotion.hpp"
#include "PoseRandomWalker.hpp"

namespace loc{
    
    template<class Ts, class Tin>
    void RandomWalkerMotion<Ts, Tin>::velocityRate(double velocityRate){
        velocityRate_ = velocityRate;
    }
    
    template<class Ts, class Tin>
    double RandomWalkerMotion<Ts, Tin>::velocityRate() const{
        return velocityRate_;
    }
    
    template<class Ts, class Tin>
    Ts RandomWalkerMotion<Ts, Tin>::predict(Ts state, Tin input){
        auto& mRandGen = RandomWalker<Ts, Tin>::mRandGen;
        const auto& mPedometer = mRWMotionProperty->pedometer();
        const auto& mOrientationMeter = mRWMotionProperty->orientationMeter();
        
        long t_pre = input.previousTimestamp();
        long t_cur = input.timestamp();
        double dt = (t_cur-t_pre)*input.timeUnit();
        
        if(dt < 0){
            std::cerr << "Inconsistent timestamp found in RandomWalkerMotion. Location was not updated." << std::endl;
            return state;
        }
        
        if(mPedometer && mOrientationMeter){
            double movLevel = movingLevel();
            double x = state.x();
            double y = state.y();
            double z = state.z();
            double floor = state.floor();
            
            if(dt<input.timeUnit()){
                throw std::runtime_error("Time increment is too small in RandomWalkerMotion.");
            }
            
            // Compute velocity rate to reduce velocity when turning
            if(mRWMotionProperty->usesAngularVelocityLimit()){
                double yaw =  Pose::normalizeOrientaion(mOrientationMeter->getYaw());
                if(!wasYawUpdated){
                    currentTimestamp = t_cur;
                    currentYaw = yaw;
                    wasYawUpdated = true;
                }
                if(currentTimestamp!=t_cur){
                    currentTimestamp = t_cur;
                    double previousYaw = Pose::normalizeOrientaion(currentYaw);
                    currentYaw = Pose::normalizeOrientaion(yaw);
                    double oriDiff = Pose::computeOrientationDifference(previousYaw, currentYaw);
                    double angularVelocity = oriDiff/dt;
                    double angularVelocityLimit = mRWMotionProperty->angularVelocityLimit();
                    turningVelocityRate = std::sqrt(1.0 - std::min(1.0, std::pow(angularVelocity/angularVelocityLimit,2)));
                }
            }else{
                turningVelocityRate = 1.0;
            }
            
            double sigma;
            if(movLevel > 0){
                sigma = mRWMotionProperty->sigmaMove;
            }else{
                sigma = mRWMotionProperty->sigmaStop;
            }
            
            // Scale sigma by time increment
            sigma = sigma * std::sqrt(1.0/dt);
            
            // Multiply sigma by velocity rate
            sigma = sigma * velocityRate();
            
            // Multyply sigma by turning velocity rate
            sigma = sigma * turningVelocityRate;
            
            double nx = mRandGen->nextGaussian();
            double ny = mRandGen->nextGaussian();
            double theta = std::atan2(ny, nx);
            
            double vx = sigma * nx;
            double vy = sigma * ny;
            double v = std::sqrt(vx*vx + vy*vy);
            
            x += vx*dt;
            y += vy*dt;
            
            Ts stateNew(state);
            stateNew.x(x).y(y).z(z).floor(floor);
            stateNew.velocity(v).normalVelocity(v).orientation(theta);
            
            return stateNew;
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
    RandomWalkerMotion<Ts, Tin>& RandomWalkerMotion<Ts, Tin>::setProperty(RandomWalkerMotionProperty::Ptr property){
        mRWMotionProperty = property;
        return *this;
    }
    
    template<class Ts, class Tin>
    double RandomWalkerMotion<Ts, Tin>::movingLevel(){
        if(isUnderControll){
            return mMovement;
        }else{
            return mRWMotionProperty->pedometer()->getNSteps();
        }
    }
    
    // Explicit instantiation
    template class RandomWalkerMotion<State, RandomWalkerInput>;
}
