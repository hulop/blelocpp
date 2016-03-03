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

#include "StatusInitializerImpl.hpp"

namespace loc{
    
    StatusInitializerImpl& StatusInitializerImpl::dataStore(std::shared_ptr<DataStore> dataStore){
        mDataStore = dataStore;
        return *this;
    }
    
    StatusInitializerImpl& StatusInitializerImpl::poseProperty(PoseProperty poseProperty){
        mPoseProperty = poseProperty;
        return *this;
    }
    
    StatusInitializerImpl& StatusInitializerImpl::stateProperty(StateProperty stateProperty){
        mStateProperty = stateProperty;
        return *this;
    }

    
    State StatusInitializerImpl::perturbRssiBias(const State &state){
        State stateNew(state);
        double rssiBias = rand.nextTruncatedGaussian(state.rssiBias(), mStateProperty.diffusionRssiBias(), mStateProperty.minRssiBias(), mStateProperty.maxRssiBias());
        stateNew.rssiBias(rssiBias);
        return stateNew;
    }
    
    template <class Tstate>
    Tstate StatusInitializerImpl::perturbLocation(const Tstate& location){
        const Building& building = mDataStore->getBuilding();
        return perturbLocation(location, building);
    }
    template Location StatusInitializerImpl::perturbLocation<Location>(const Location& location);
    template Pose StatusInitializerImpl::perturbLocation<Pose>(const Pose& location);
    template State StatusInitializerImpl::perturbLocation<State>(const State& location);
    
    template <class Tstate>
    Tstate StatusInitializerImpl::perturbLocation(const Tstate& location, const Building& building){
        bool hasBuilding = building.nFloors()>0? true: false;
        for(int i=0; i<nPerturbationMax; i++){
            Tstate locNew(location);
            double x = locNew.x() + mPoseProperty.stdX() * rand.nextGaussian();
            double y = locNew.y() + mPoseProperty.stdY() * rand.nextGaussian();
            locNew.x(x);
            locNew.y(y);
            
            if(hasBuilding){
                if(building.isMovable(locNew)){
                    return locNew;
                }
            }else{
                return locNew;
            }
        }
        return location;
    }
    
    Locations StatusInitializerImpl::extractMovableLocations(const Locations& locations){
        const Building& building = mDataStore->getBuilding();
        std::vector<Location> movableLocations;
        
        // Filter movable points
        bool hasBuilding = building.nFloors()>0? true : false;
        if(hasBuilding){
            for(auto loc: locations){
                if(building.isMovable(loc)){
                    movableLocations.push_back(Location(loc));
                }
            }
        }else{
            movableLocations.insert(movableLocations.end(), locations.begin(), locations.end());
        }
        return movableLocations;
    }
    
    Locations StatusInitializerImpl::randomSampleLocationsWithPerturbation(int n, const Locations& locations){
        Locations locs;
        int nSamples = (int) locations.size();
        assert(nSamples>0);
        std::vector<int> indices = rand.randomSet(nSamples, n);
        for(int i=0; i<n; i++){
            int idx = indices.at(i);
            //std::cout << "idx=" << idx << std::endl;
            Location loc = locations.at(idx);
            loc = perturbLocation(loc);
            locs.push_back(Location(loc));
        }
        return locs;
    }
    
    Locations StatusInitializerImpl::initializeLocations(int n){
        assert(n>0);
        
        const Samples& samples = mDataStore->getSamples();
        
        std::vector<Location> uniqueLocations = Sample::extractUniqueLocations(samples);
        std::vector<Location> movableLocations = extractMovableLocations(uniqueLocations);
        
        // Filter movable points
        int countNonMovable = uniqueLocations.size() - movableLocations.size();

        std::cout << "Invalid " << countNonMovable << " points are not used for initialization." << std::endl;
        
        // Random sampling
        Locations locs = randomSampleLocationsWithPerturbation(n, movableLocations);
        assert(locs.size()==n);
        
        // Check if all locations are valid.
        const Building& building = mDataStore->getBuilding();
        bool hasBuilding = building.nFloors()>0? true : false;
        if(hasBuilding){
            for(Location loc: locs){
                assert(building.isMovable(loc));
            }
        }else{
            // pass
        }
        return locs;
    }
    
    Poses StatusInitializerImpl::initializePosesFromLocations(Locations locs){
        size_t n = locs.size();
        Poses poses(n);
        for(int i=0; i<n; i++){
            Location loc = locs.at(i);
            Pose pose(loc);
            
            double orientation = 2.0*M_PI*rand.nextDouble();
            double velocity = 0.0;
            double normalVelocity = rand.nextTruncatedGaussian(mPoseProperty.meanVelocity(), mPoseProperty.stdVelocity(), mPoseProperty.minVelocity(), mPoseProperty.maxVelocity());
            
            pose.orientation(orientation)
            .velocity(velocity)
            .normalVelocity(normalVelocity);
            poses[i]=pose;
        }
        return poses;
    }
    
    
    Poses StatusInitializerImpl::initializePoses(int n){
        Locations locs = initializeLocations(n);
        return initializePosesFromLocations(locs);
    }
    
    
    States StatusInitializerImpl::initializeStatesFromPoses(Poses poses){
        size_t n = poses.size();
        States states(n);
        for(int i=0; i<n; i++){
            Pose pose = poses.at(i);
            State state(pose);
            
            double orientationBias = 2.0*M_PI*rand.nextDouble();
            double rssiBias = rand.nextTruncatedGaussian(mStateProperty.meanRssiBias(), mStateProperty.stdRssiBias(), mStateProperty.minRssiBias(), mStateProperty.maxRssiBias());
            
            state.orientationBias(orientationBias).rssiBias(rssiBias);
            state.weight(1.0/n);
            states[i]=state;
        }
        return states;
    }
    
    
    States StatusInitializerImpl::initializeStatesFromLocations(const std::vector<Location> &locations){
        const Poses& poses = initializePosesFromLocations(locations);
        return initializeStatesFromPoses(poses);
    }
    
    
    States StatusInitializerImpl::initializeStates(int n){
        Poses poses = initializePoses(n);
        return initializeStatesFromPoses(poses);
    }
    
    States StatusInitializerImpl::resetStates(int n, Pose pose, double orientationMeasured){
        
        double xReset = pose.x();
        double yReset = pose.y();
        double zReset = pose.z();
        double floorReset = pose.floor();
        double orientationReset = pose.orientation();
        double orientationBiasReset = orientationMeasured - orientationReset;

        //std::cout << "Reset status(PoseReset="<<pose<<", yaw="<<orientationMeasured<< ",orientationBias=" << orientationBiasReset << std::endl;
        
        States states = initializeStates(n);
        for(int i=0; i<n; i++){
            State state = states.at(i);
            state.x(xReset).y(yReset).z(zReset).floor(floorReset).orientation(orientationReset);
            state.orientationBias(orientationBiasReset);
            state.weight(1.0/n);
            states[i] = state;
        }
        
        return states;
    }
    
    States StatusInitializerImpl::resetStates(int n,  Pose meanPose, Pose stdevPose, double orientationMeasured){
        
        Building building = mDataStore->getBuilding();
        
        if(building.nFloors()>0){
            assert(building.isValid(meanPose));
            assert(!building.isWall(meanPose));
            assert(building.isMovable(meanPose));
        }
        
        States statesTmp = resetStates(n, meanPose, orientationMeasured);
        States states(n);
        for(int i=0; i<n; ){
            while(true){
                State s = statesTmp.at(i);
                double x = s.x() + stdevPose.x()*rand.nextGaussian();
                double y = s.y() + stdevPose.y()*rand.nextGaussian();
                double z = s.z() + stdevPose.z()*rand.nextGaussian();
                double floor = s.floor() + stdevPose.floor()*rand.nextGaussian();
                double orientation = s.orientation() + stdevPose.orientation()*rand.nextGaussian();
                double orientationBias = orientationMeasured - s.orientation();

                // State stateNew(s);
                s.x(x).y(y).z(z).floor(floor).orientation(orientation);
                s.orientationBias(orientationBias);
                
                // only if meanPose.normalVelocity is valid, normalVelocity is updated.
                if(mPoseProperty.minVelocity() < meanPose.normalVelocity()
                   && meanPose.normalVelocity() < mPoseProperty.maxVelocity()){
                    double normalVelocity = rand.nextTruncatedGaussian(meanPose.normalVelocity(), stdevPose.normalVelocity(), mPoseProperty.minVelocity(), mPoseProperty.maxVelocity());
                    s.normalVelocity(normalVelocity);
                }
                
                if(building.nFloors()>0){
                    if(building.isMovable(s)){
                        states[i] = s;
                        i++;
                        break;
                    }
                }else{
                    states[i] = s;
                    i++;
                    break;
                }
            }
        }
        return states;
    }
    
    
    void StatusInitializerImpl::beaconEffectiveRadius2D(double radius2D){
        mRadius2D = radius2D;
    }
    
    States StatusInitializerImpl::resetStates(int n, const std::vector<Beacon>& beacons){
        Locations locs = extractLocationsCloseToBeacons(beacons, mRadius2D);
        Locations movableLocs = extractMovableLocations(locs);
        Locations selectedLocs = randomSampleLocationsWithPerturbation(n, movableLocs);
        return initializeStatesFromLocations(selectedLocs);
    }
    
    
    Locations StatusInitializerImpl::extractLocationsCloseToBeacons(const std::vector<Beacon> &beacons, double radius2D) const{
        
        auto samples = mDataStore->getSamples();
        auto bleBeacons = mDataStore->getBLEBeacons();
        
        std::map<long, int> idToIndexMap = BLEBeacon::constructBeaconIdToIndexMap(bleBeacons);
        std::vector<Location> selectedLocations;

        std::vector<BLEBeacon> observedBLEBeacons;
        for(auto& b: beacons){
            long id = b.id();
            if(idToIndexMap.count(id)>0){
                observedBLEBeacons.push_back(bleBeacons.at(idToIndexMap.at(id)));
            }
        }
        
        auto locations = Sample::extractUniqueLocations(samples);
        for(auto& loc: locations){
            for(auto& bloc: observedBLEBeacons){
                double dist = Location::distance2D(loc, bloc);
                double floorDiff = Location::floorDifference(loc, bloc);
                if(dist <= radius2D && floorDiff==0){
                    selectedLocations.push_back(loc);
                    continue;
                }
            }
        }
        
        return selectedLocations;
    }
    
    
}