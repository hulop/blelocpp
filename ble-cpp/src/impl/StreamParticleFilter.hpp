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
#include "ObservationDependentInitializer.hpp"
#include "PosteriorResampler.hpp"

#include "BeaconFilter.hpp"
#include "AltitudeManager.hpp"

namespace loc {
    
    enum FloorUpdateMode{
        COUNT,
        WEIGHT
    };
    
    class LocationStatusMonitorParameters{
    protected:
        double minimumWeightStable_ = 1.0e-5;
        
        double stdev2DEnterStable_ = 5.0;
        double stdev2DExitStable_ = 10.0;
        double stdevFloorEnterStable_ = 0.1;
        
        double stdev2DEnterLocating_ = 8.0;
        double stdev2DExitLocating_ = 10.0;
        
    public:
        using Ptr = std::shared_ptr<LocationStatusMonitorParameters>;
        double minimumWeightStable() const {return minimumWeightStable_;}
        double stdev2DEnterStable() const {return stdev2DEnterStable_;}
        double stdev2DExitStable() const {return stdev2DExitStable_;}
        double stdev2DEnterLocating() const{return stdev2DEnterLocating_;}
        double stdev2DExitLocating() const{return stdev2DExitLocating_; }
        void minimumWeightStable(double weight){minimumWeightStable_=weight;}
        void stdev2DEnterStable(double stdev2D) {stdev2DEnterStable_ = stdev2D;}
        void stdev2DExitStable(double stdev2D){stdev2DExitStable_ = stdev2D;}
        void stdev2DEnterLocating(double stdev2D){stdev2DEnterLocating_ = stdev2D;}
        void stdev2DExitLocating(double stdev2D){stdev2DExitLocating_ = stdev2D; }
    };
    
    //template<class Tsys, class Tobs>
    class StreamParticleFilter : public StreamLocalizer{
    public:
        class MixtureParameters{
            double mRejectFloorDifference=0.5; // Threshold of the probability that the floor of the states and a state to be mixed is different.
        public:
            double mixtureProbability=0;
            double rejectDistance=5; // Threshold of the distance between the states and a state to be mixed
            int burnInQuick = 50;
            int nBeaconsMinimum = 3; // The minimum number of observed beacons to apply mix.
            
            double rejectFloorDifference() const{return mRejectFloorDifference;}
            void rejectFloorDifference(double rejFloorDiff){mRejectFloorDifference = rejFloorDiff;}
        };
        
        class FloorTransitionParameters{
        protected:
            double heightChangedCriterion_ = 0.0;
            double weightTransitionArea_ = 1.0;
            double mixtureProbaTransArea_ = 0.0;
            double rejectDistance_ = 5.0;
            
        public:
            using Ptr = std::shared_ptr<FloorTransitionParameters>;
            double heightChangedCriterion() const;
            double weightTransitionArea() const;
            FloorTransitionParameters& heightChangedCriterion(double);
            FloorTransitionParameters& weightTransitionArea(double);
            
            double mixtureProbaTransArea() const;
            double rejectDistance() const;
            FloorTransitionParameters& mixtureProbaTransArea(double);
            FloorTransitionParameters& rejectDistance(double);
        };
        
        StreamParticleFilter();
        ~StreamParticleFilter();
        
        // setter
        StreamParticleFilter& optVerbose(bool);
        StreamParticleFilter& numStates(int);
        StreamParticleFilter& alphaWeaken(double);
        StreamParticleFilter& effectiveSampleSizeThreshold(double);
        StreamParticleFilter& mixtureParameters(MixtureParameters);
        StreamParticleFilter& floorTransitionParameters(FloorTransitionParameters::Ptr);
        StreamParticleFilter& enablesFloorUpdate(bool);
        StreamParticleFilter& floorUpdateMode(FloorUpdateMode);
        StreamParticleFilter& locationStatusMonitorParameters(LocationStatusMonitorParameters::Ptr);
        
        StreamParticleFilter& locationStandardDeviationLowerBound(Location loc);
        
        StreamParticleFilter& pedometer(std::shared_ptr<Pedometer> pedometer);
        StreamParticleFilter& orientationMeter(std::shared_ptr<OrientationMeter>  orientationMeter);
        StreamParticleFilter& altitudeManager(std::shared_ptr<AltitudeManager>  altitudeManager);
        StreamParticleFilter& statusInitializer(std::shared_ptr<StatusInitializer> statusInitializer);
        StreamParticleFilter& systemModel(std::shared_ptr<SystemModel<State, SystemModelInput>> poseRandomWalker);
        StreamParticleFilter& observationModel(std::shared_ptr<ObservationModel<State, Beacons>> observationModel);
        StreamParticleFilter& resampler(std::shared_ptr<Resampler<State>> resampler);
        
        StreamParticleFilter& beaconFilter(std::shared_ptr<BeaconFilter> beaconFilter);
        
        StreamParticleFilter& observationDependentInitializer(std::shared_ptr<ObservationDependentInitializer<State, Beacons>> metro);
        StreamParticleFilter& posteriorResampler(PosteriorResampler<State>::Ptr);
        StreamParticleFilter& dataStore(DataStore::Ptr);
        
        // callback function setter
        StreamParticleFilter& updateHandler(void (*functionCalledAfterUpdate)(Status*)) override;
        StreamParticleFilter& updateHandler(void (*functionCalledAfterUpdate)(void*, Status*), void* inUserData) override;
        
        // method to input sensor data
        StreamParticleFilter& putAcceleration(const Acceleration acceleration) override;
        StreamParticleFilter& putAttitude(const Attitude attitude) override;
        StreamParticleFilter& putBeacons(const Beacons beacons) override;
        StreamParticleFilter& putLocalHeading(const LocalHeading heading) override;
        StreamParticleFilter& putAltimeter(const Altimeter altimeter) override;
        Status* getStatus() override;
        
        // optional methods
        bool resetStatus() override;
        bool resetStatus(Pose pose) override;
        bool resetStatus(Pose meanPose, Pose stdevPose) override;
        bool resetStatus(Pose meanPose, Pose stdevPose, double rateContami);
        bool resetStatus(const Beacons& beacons) override;
        bool resetStatus(const Location& location, const Beacons& beacons) override;
        
        // (unstable functions)
        // Call this function to search initial location
        bool refineStatus(const Beacons& beacons);
        
    private:
        class Impl;
        std::shared_ptr<Impl> impl;
    };
}

#endif /* StreamParticleFilter_hpp */
