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
    
    PoseRandomWalkerInBuildingProperty& PoseRandomWalkerInBuildingProperty::velocityRateEscalator(double velocityRateEscalator){
        velocityRateEscalator_ = velocityRateEscalator;
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
    
    double PoseRandomWalkerInBuildingProperty::velocityRateEscalator() const{
        return velocityRateEscalator_;
    }
    
    double PoseRandomWalkerInBuildingProperty::weightDecayRate() const{
        return weightDecayRate_;
    }
    
    int PoseRandomWalkerInBuildingProperty::maxTrial() const{
        return maxTrial_;
    }
}

