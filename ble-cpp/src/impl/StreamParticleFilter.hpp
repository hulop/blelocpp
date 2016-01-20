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

#ifndef StreamParticleFilter_hpp
#define StreamParticleFilter_hpp

#include <stdio.h>
#include "bleloc.h"
#include "StreamLocalizer.hpp"

#include "StatusInitializer.hpp"
#include "PoseRandomWalker.hpp"
#include "ObservationModel.hpp"
#include "Resampler.hpp"

#include "BeaconFilter.hpp"

namespace loc {
    //template<class Tsys, class Tobs>
    class StreamParticleFilter : public StreamLocalizer{
    public:
        StreamParticleFilter();
        ~StreamParticleFilter();
        
        // setter
        StreamParticleFilter& optVerbose(bool);
        StreamParticleFilter& numStates(int);
        StreamParticleFilter& alphaWeaken(double);
        StreamParticleFilter& locationStandardDeviationLowerBound(Location loc);
        
        StreamParticleFilter& pedometer(std::shared_ptr<Pedometer> pedometer);
        StreamParticleFilter& orientationMeter(std::shared_ptr<OrientationMeter>  orientationMeter);
        StreamParticleFilter& statusInitializer(std::shared_ptr<StatusInitializer> statusInitializer);
        StreamParticleFilter& systemModel(std::shared_ptr<SystemModel<State, PoseRandomWalkerInput>> poseRandomWalker);
        StreamParticleFilter& observationModel(std::shared_ptr<ObservationModel<State, Beacons>> observationModel);
        StreamParticleFilter& resampler(std::shared_ptr<Resampler<State>> resampler);
        
        StreamParticleFilter& beaconFilter(std::shared_ptr<BeaconFilter> beaconFilter);
        
        // callback function setter
        StreamParticleFilter& updateHandler(void (*functionCalledAfterUpdate)(Status*)) override;
        StreamParticleFilter& updateHandler(void (*functionCalledAfterUpdate)(void*, Status*), void* inUserData) override;
        
        // method to input sensor data
        StreamParticleFilter& putAcceleration(const Acceleration acceleration) override;
        StreamParticleFilter& putAttitude(const Attitude attitude) override;
        StreamParticleFilter& putBeacons(const Beacons beacons) override;
        Status* getStatus() override;
        
        // optional methods
        bool resetStatus() override;
        bool resetStatus(Pose pose) override;
        bool resetStatus(Pose meanPose, Pose stdevPose) override;
        
        // (unstable functions)
        // Call this function to search initial location
        bool refineStatus(const Beacons& beacons);
        
    private:
        class Impl;
        std::shared_ptr<Impl> impl;
    };
}

#endif /* StreamParticleFilter_hpp */
