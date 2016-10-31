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

#ifndef PoseRandomWalkerInBuilding_hpp
#define PoseRandomWalkerInBuilding_hpp

#include <stdio.h>
#include "PoseRandomWalker.hpp"
#include "Building.hpp"

namespace loc{
    
    class PoseRandomWalkerInBuildingProperty{
        double probabilityUp_ = 0.25;
        double probabilityDown_ = 0.25;
        double probabilityStay_ = 0.5;
        
        double wallCrossingAliveRate_ = 1.0; // fixed value
        double maxIncidenceAngle_ = 45/180*M_PI;
        
        // Field velocity rate
        double velocityRateFloor_ = 1.0;
        double velocityRateStair_ = 0.5;
        double velocityRateElevator_ = 0.5;
        //double velocityRateEscalator_ = 0.5; //0.5 is too small.
        double velocityRateEscalator_ = 1.0;
        
        double weightDecayRate_ = 0.9;
        
        int maxTrial_ = 1; // fixed value
        
    public:
        
        using Ptr = std::shared_ptr<PoseRandomWalkerInBuildingProperty>;
        
        PoseRandomWalkerInBuildingProperty& probabilityUp(double probabilityUp);
        PoseRandomWalkerInBuildingProperty& probabilityDown(double probabilityDown);
        PoseRandomWalkerInBuildingProperty& probabilityStay(double probabilityStay);
        PoseRandomWalkerInBuildingProperty& wallCrossingAliveRate(double wallCrossingAliveRate);
        PoseRandomWalkerInBuildingProperty& maxIncidenceAngle(double maxIncidenceAngle);
        
        PoseRandomWalkerInBuildingProperty& velocityRateFloor(double velocityRateFloor);
        PoseRandomWalkerInBuildingProperty& velocityRateStair(double velocityRateStair);
        PoseRandomWalkerInBuildingProperty& velocityRateElevator(double velocityRateElevator);
        PoseRandomWalkerInBuildingProperty& velocityRateEscalator(double velocityRateEscalator);
        
        PoseRandomWalkerInBuildingProperty& weightDecayRate(double weightDecayRate);
        double probabilityUp() const;
        double probabilityDown() const;
        double probabilityStay() const;
        double wallCrossingAliveRate() const;
        double maxIncidenceAngle() const;
        double velocityRateFloor() const;
        double velocityRateStair() const;
        double velocityRateElevator() const;
        double velocityRateEscalator() const;
        double weightDecayRate() const;
        int maxTrial() const;
        
    };
}

#endif /* PoseRandomWalkerInBuilding_hpp */
