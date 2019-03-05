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

#ifndef StatusInitializerImpl_hpp
#define StatusInitializerImpl_hpp

#include <stdio.h>
#include <cmath>
#include <memory>

#include "RandomGenerator.hpp"
#include "StatusInitializer.hpp"
#include "DataStore.hpp"

namespace loc{
    /**
     sampling points based status initializer
    **/
    
    class StatusInitializerImpl : public StatusInitializer{
        
    private:
        RandomGenerator rand;
        std::shared_ptr<DataStore> mDataStore;
        
        template<class Tstate>
        Tstate perturbLocation(const Tstate& location, const Building& building);
        template<class Tstate>
        Tstate perturbLocation(const Tstate& location, double stdx, double stdy, const Building& building);
        PoseProperty::Ptr mPoseProperty = PoseProperty::Ptr(new PoseProperty);
        StateProperty::Ptr mStateProperty = StateProperty::Ptr(new StateProperty);
        
        Poses initializePosesFromLocations(Locations locations);
        States initializeStatesFromPoses(Poses poses);
        
    public:
        int nPerturbationMax = 100;
        double mRadius2D = 10; //[m]

        void beaconEffectiveRadius2D(double);
        StatusInitializerImpl& dataStore(std::shared_ptr<DataStore> dataStore);
        StatusInitializerImpl& poseProperty(PoseProperty::Ptr poseProperty);
        StatusInitializerImpl& stateProperty(StateProperty::Ptr stateProperty);
        
        template<class Tstate>
        Tstate perturbLocation(const Tstate& location);
        
        template<class Tstate>
        Tstate perturbLocation(const Tstate& location, double stdx, double stdy);
        
        template<class Tstate>
        std::vector<Tstate> perturbLocations(const std::vector<Tstate>& locations);
        
        State perturbRssiBias(const State& state);
        
        Locations initializeLocations(int n);
        Locations extractMovableLocations(const Locations& locations);
        Locations randomSampleLocationsWithPerturbation(int n, const Locations& locations);
        Poses initializePoses(int n);
        States initializeStates(int n);

        States resetStates(int n, Pose pose, double orientationMeasured);
        [[deprecated("please use resetStates(int n, Pose meanPose, State stdevState, double orientationMeasured)")]]
        States resetStates(int n, Pose meanPose, Pose stdevPose, double orientationMeasured);
        States resetStates(int n, Pose meanPose, State stdevState, double orientationMeasured);
        States resetStates(int n, const std::vector<Beacon>& beacons);
        
        States initializeStatesFromLocations(const std::vector<Location>& locations);
        Locations extractLocationsCloseToBeacons(const std::vector<Beacon>& beacons, double radius2D, double floorDiff = 0.0) const;
        Locations generateLocationsCloseToBeaconsWithPerturbation(const std::vector<Beacon> &beacons, double radius2D);
    };
    
    // Implementation
    template <class Tstate>
    std::vector<Tstate> StatusInitializerImpl::perturbLocations(const std::vector<Tstate>& locations){
        std::vector<Tstate> locsNew;
        for(const Tstate& loc: locations){
            Tstate locNew = perturbLocation(loc);
            locsNew.push_back(locNew);
        }
        return locsNew;
    }
    
}

#endif /* StatusInitializerImpl_hpp */
