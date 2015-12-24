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

#include <iostream>
#include "PoseRandomWalkerInBuilding.hpp"

namespace loc{
    
    // PoseRandomWalkerInBuildingProperty
    PoseRandomWalkerInBuildingProperty& PoseRandomWalkerInBuildingProperty::probabilityUp(double probabilityUp){
        probabilityUp_ = probabilityUp;
        return *this;
    }
    
    PoseRandomWalkerInBuildingProperty& PoseRandomWalkerInBuildingProperty::probabilityDown(double probabilityDown){
        probabilityDown_ = probabilityDown;
        return *this;
    }
    
    PoseRandomWalkerInBuildingProperty& PoseRandomWalkerInBuildingProperty::probabilityStay(double probabilityStay){
        probabilityStay_ = probabilityStay;
        return *this;
    }
    
    PoseRandomWalkerInBuildingProperty& PoseRandomWalkerInBuildingProperty::wallCrossingAliveRate(double wallCrossingAliveRate){
        wallCrossingAliveRate_ = wallCrossingAliveRate;
        return *this;
    }
    
    PoseRandomWalkerInBuildingProperty& PoseRandomWalkerInBuildingProperty::maxIncidenceAngle(double maxIncidenceAngle){
        maxIncidenceAngle_ = maxIncidenceAngle;
        return *this;
    }
    
    PoseRandomWalkerInBuildingProperty& PoseRandomWalkerInBuildingProperty::velocityRateFloor(double velocityRateFloor){
        velocityRateFloor_ = velocityRateFloor;
        return *this;
    }
    
    PoseRandomWalkerInBuildingProperty& PoseRandomWalkerInBuildingProperty::velocityRateStair(double velocityRateStair){
        velocityRateStair_ = velocityRateStair;
        return *this;
    }
    
    PoseRandomWalkerInBuildingProperty& PoseRandomWalkerInBuildingProperty::velocityRateElevator(double velocityRateElevator){
        velocityRateElevator_ = velocityRateElevator;
        return *this;
    }
    
    PoseRandomWalkerInBuildingProperty& PoseRandomWalkerInBuildingProperty::weightDecayRate(double weightDecayRate){
        weightDecayRate_ = weightDecayRate;
        return *this;
    }
    
    double PoseRandomWalkerInBuildingProperty::probabilityUp() const{
        return probabilityUp_;
    }
    
    double PoseRandomWalkerInBuildingProperty::probabilityDown() const{
        return probabilityDown_;
    }
    
    double PoseRandomWalkerInBuildingProperty::probabilityStay() const{
        return probabilityStay_;
    }
    
    double PoseRandomWalkerInBuildingProperty::wallCrossingAliveRate() const{
        return wallCrossingAliveRate_;
    }
    
    double PoseRandomWalkerInBuildingProperty::maxIncidenceAngle() const{
        return maxIncidenceAngle_;
    }
    
    double PoseRandomWalkerInBuildingProperty::velocityRateFloor() const{
        return velocityRateFloor_;
    }
    
    double PoseRandomWalkerInBuildingProperty::velocityRateStair() const{
        return velocityRateStair_;
    }
    
    double PoseRandomWalkerInBuildingProperty::velocityRateElevator() const{
        return velocityRateElevator_;
    }
    
    double PoseRandomWalkerInBuildingProperty::weightDecayRate() const{
        return weightDecayRate_;
    }
    
    int PoseRandomWalkerInBuildingProperty::maxTrial() const{
        return maxTrial_;
    }
    
    
    // PoseRandomWalkerInBuilding
    PoseRandomWalkerInBuilding::PoseRandomWalkerInBuilding(PoseRandomWalker poseRandomWalker, Building building, PoseRandomWalkerInBuildingProperty property){
        
        mPoseRandomWalker = poseRandomWalker;
        mBuilding = building;
        mProperty = property;
    }
    
    
    PoseRandomWalkerInBuilding& PoseRandomWalkerInBuilding::poseRandomWalker(PoseRandomWalker poseRandomWalker){
        mPoseRandomWalker = poseRandomWalker;
        return *this;
    }
    
    PoseRandomWalkerInBuilding& PoseRandomWalkerInBuilding::building(Building building){
        mBuilding = building;
        return *this;
    }
    
    PoseRandomWalkerInBuilding& PoseRandomWalkerInBuilding::poseRandomWalkerInBuildingProperty(PoseRandomWalkerInBuildingProperty property){
        mProperty = property;
        return *this;
    }
    
    
    State PoseRandomWalkerInBuilding::moveOnElevator(const State& state, PoseRandomWalkerInput input){
        int f_min = mBuilding.minFloor();
        int f_max = mBuilding.maxFloor();
        State stateNew;
        while(true){
            int f_new = f_min + mRandomGenerator.nextInt(f_max - f_min);
            assert(mBuilding.isValidFloor(f_new));
            if(mBuilding.isValidFloor(f_new)){
                stateNew = State(state);
                stateNew.floor(f_new);
                if(mBuilding.isMovable(stateNew) && mBuilding.isElevator(stateNew)){
                    break;
                }
            }
        }
        assert(mBuilding.isMovable(stateNew));
        return stateNew;
    }
    
    State PoseRandomWalkerInBuilding::moveOnStair(const State& state, PoseRandomWalkerInput input){
        int f_min = mBuilding.minFloor();
        int f_max = mBuilding.maxFloor();
        int f = state.floor();
        int f_new;
        
        double pUp = mProperty.probabilityUp();
        double pDown = mProperty.probabilityDown();
        double pStay = mProperty.probabilityStay();
        
        if(f==f_min){
            pDown = 0;
            pUp = mProperty.probabilityDown()/(mProperty.probabilityDown() + mProperty.probabilityStay());
            pStay = 1 - pUp;
        }else if(f==f_max){
            pDown = mProperty.probabilityUp()/(mProperty.probabilityUp() + mProperty.probabilityStay());
            pUp = 0;
            pStay = 1 - pDown;
        }
        
        State stateNew(state);
        while(true){
            double p = mRandomGenerator.nextDouble();
            if(p < pUp){
                f_new = f+1;
            }else if( p - pUp < pDown){
                f_new = f-1;
            }else{
                f_new = f;
            }
            assert(mBuilding.isValidFloor(f_new));
            if(mBuilding.isValidFloor(f_new)){
                stateNew = State(state);
                stateNew.floor(f_new);
                if(mBuilding.isMovable(stateNew) && mBuilding.isStair(stateNew)){
                    break;
                }
            }
        }
        assert(mBuilding.isMovable(stateNew));
        return stateNew;
    }
    
    State moveWithAngle(const State& state, const PoseRandomWalkerInput& input, double angle){
        State stateNew(state);
        double dt= (input.timestamp() - input.previousTimestamp())/(1000.0);
        double vx = stateNew.velocity() * std::cos(angle);
        double vy = stateNew.velocity() * std::sin(angle);
        stateNew.Location::x(state.Location::x() + vx*dt);
        stateNew.Location::y(state.Location::y() + vy*dt);
        return stateNew;
    }
    
    State PoseRandomWalkerInBuilding::moveOnFloor(const State& state, PoseRandomWalkerInput input){
        assert(mBuilding.isMovable(state));
        State stateNew(state);
        // Change field velocity rate
        if(mBuilding.isElevator(state)){
            mPoseRandomWalker.velocityRate(mProperty.velocityRateElevator());
        }else if(mBuilding.isStair(state)){
            mPoseRandomWalker.velocityRate(mProperty.velocityRateStair());
        }else{
            mPoseRandomWalker.velocityRate(mProperty.velocityRateFloor());
        }
        // Update state
        for(int i=0; i<mProperty.maxTrial() ; i++){
            stateNew = mPoseRandomWalker.predict(state, input);
            if(mBuilding.checkMovableRoute(state, stateNew)){
                break;
            }else if(i==mProperty.maxTrial()-1){
                stateNew = moveOnFloorRetry(state, stateNew, input);
                assert(mBuilding.checkMovableRoute(state, stateNew));
            }
        }
        mPoseRandomWalker.velocityRate(mProperty.velocityRateFloor());
        assert(mBuilding.isMovable(stateNew) || stateNew.weight()==0.0);
        return stateNew;
    }
    
    State PoseRandomWalkerInBuilding::moveOnFloorRetry(const State& state, const State& stateNew, PoseRandomWalkerInput input){
        
        State stateTmp(stateNew);
        if( mRandomGenerator.nextDouble() < mProperty.wallCrossingAliveRate()){
            double orientation = atan2(stateNew.y() - state.y(), stateNew.x() - state.x());
            double angle = mBuilding.estimateWallAngle(state, stateNew);
            if(std::abs(Pose::computeOrientationDifference(orientation, angle))
               < mProperty.maxIncidenceAngle()){
                stateTmp = moveWithAngle(state, input, angle);
            }
            if(mBuilding.checkMovableRoute(state, stateTmp)){
                return stateTmp;
            }else{
                stateTmp = State(state);
                stateTmp.weight(state.weight() * mProperty.weightDecayRate());
            }
        }
        
        assert(mBuilding.isMovable(stateTmp));
        return stateTmp;
    }
    
    State PoseRandomWalkerInBuilding::predict(State state, PoseRandomWalkerInput input){
        assert(mBuilding.isMovable(state));
        if(mBuilding.isElevator(state)){
            State stateTmp = moveOnElevator(state, input);
            if(Location::floorDifference(state, stateTmp)==0){
                return moveOnFloor(stateTmp, input);
            }else{
                return stateTmp;
            }
        }else if(mBuilding.isStair(state)){
            State stateTmp = moveOnStair(state, input);
            return moveOnFloor(stateTmp, input);
        }else{
            return moveOnFloor(state, input);
        }
    }
    
    
    std::vector<State> PoseRandomWalkerInBuilding::predict(std::vector<State> states, PoseRandomWalkerInput input){
        std::vector<State> statesPredicted(states.size());
        for(int i=0; i<states.size(); i++){
            const State& state = states[i];
            statesPredicted[i]= predict(state, input);
        }
        return statesPredicted;
    }
    
}