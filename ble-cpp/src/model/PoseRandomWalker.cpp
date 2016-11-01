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

#include "PoseRandomWalker.hpp"


namespace loc{
    
    PoseRandomWalker& PoseRandomWalker::setProperty(PoseRandomWalkerProperty property){
        mProperty = property;
        return *this;
    }
    
    PoseRandomWalker& PoseRandomWalker::setPoseProperty(loc::PoseProperty poseProperty){
        this->poseProperty = poseProperty;
        return *this;
    }
    
    PoseRandomWalker& PoseRandomWalker::setStateProperty(loc::StateProperty stateProperty){
        this->stateProperty = stateProperty;
        return *this;
    }
    
    std::vector<State> PoseRandomWalker::predict(std::vector<State> states, SystemModelInput input){
        std::vector<State> statesPredicted(states.size());
        for(int i=0; i<states.size(); i++){
            statesPredicted[i]= predict(states[i], input);
        }
        return statesPredicted;
    }
    
    State PoseRandomWalker::predict(State state, SystemModelInput input){
        
        //long timestamp = input.timestamp;
        //long previousTimestamp = input.previousTimestamp;
        double dTime = (input.timestamp()-input.previousTimestamp())/(1000.0); //[s] Difference in time
        
        double movLevel = movingLevel();
        double yaw = mProperty.orientationMeter()->getYaw();
        
        //std::cout << "predict: dTime=" << dTime << ", nSteps=" << nSteps << std::endl;
        
        // Perturb variables in State
        if(movLevel>0 || mProperty.doesUpdateWhenStopping() ){
            state.orientationBias(state.orientationBias() + stateProperty.diffusionOrientationBias()*randomGenerator.nextGaussian()*dTime );
            state.rssiBias(randomGenerator.nextTruncatedGaussian(state.rssiBias(), stateProperty.diffusionRssiBias()*dTime , stateProperty.minRssiBias(), stateProperty.maxRssiBias()));
        }
        
        // Update orientation
        double previousOrientation = state.orientation();
        double orientationActual = yaw - state.orientationBias();
        orientationActual += poseProperty.stdOrientation()*randomGenerator.nextGaussian()*dTime;
        orientationActual = Pose::normalizeOrientaion(orientationActual);
        state.orientation(orientationActual);
        
        // Reduce velocity when turning
        double oriDiff = Pose::computeOrientationDifference(previousOrientation, orientationActual);
        double angularVelocityLimit = mProperty.angularVelocityLimit();
        double turningVelocityRate = std::sqrt(1.0 - std::min(1.0, std::pow(oriDiff/angularVelocityLimit,2)));
        
        
        // Perturb variables in Pose
        double v = 0.0;
        double nV = state.normalVelocity();
        if(movLevel >0 || mProperty.doesUpdateWhenStopping()){
            nV = randomGenerator.nextTruncatedGaussian(state.normalVelocity(),
                                                          poseProperty.diffusionVelocity(),
                                                          poseProperty.minVelocity(),
                                                          poseProperty.maxVelocity());
            state.normalVelocity(nV);
        }
        if(movLevel>0){
            v = nV * mVelocityRate * turningVelocityRate;
        }else{
            v = 0.0;
        }
        state.velocity(v);
        
        // Update in-plane coordinate.
        double x = state.x() + state.vx() * dTime;
        double y = state.y() + state.vy() * dTime;
        
        State statePred(state);
        statePred.x(x);
        statePred.y(y);
        
        return statePred;
    }

    
    double PoseRandomWalker::movingLevel(){
        if(isUnderControll){
            return mMovement;
        }else{
            return mProperty.pedometer()->getNSteps();
        }
    }
    
}
