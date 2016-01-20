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

#include <thread>
#include <queue>
#include <functional>

#include "StreamParticleFilter.hpp"
#include "StreamLocalizer.hpp"

#include "ObservationModel.hpp"

#include "SystemModel.hpp"

#include "Resampler.hpp"
#include "GridResampler.hpp"

#include "StatusInitializer.hpp"

#include "PoseRandomWalker.hpp"

#include "ArrayUtils.hpp"

#include "DataStore.hpp"
#include "DataLogger.hpp"
#include "BaseBeaconFilter.hpp"
#include "CleansingBeaconFilter.hpp"

namespace loc{
    
    class StreamParticleFilter::Impl{

    private:
        bool mOptVerbose = false;
        int mNumStates = 1000; //Defalt value
        double mAlphaWeaken = 1.0;
        int resetWaitingTimeMS = 100; // milliseconds
        long previousTimestampMotion = 0;
        Location mLocStdevLB;
        
        //std::queue<Pose> posesForReset;
        std::queue<std::function<void()>> functionsForReset;
        
        std::shared_ptr<Status> status;
        
        std::shared_ptr<Pedometer> mPedometer;
        std::shared_ptr<OrientationMeter> mOrientationmeter;
        
        std::shared_ptr<SystemModel<State, PoseRandomWalkerInput>> mRandomWalker;
        
        std::shared_ptr<ObservationModel<State, Beacons>> mObservationModel;
        std::shared_ptr<Resampler<State>> mResampler;
        std::shared_ptr<StatusInitializer> mStatusInitializer;
        std::shared_ptr<BeaconFilter> mBeaconFilter;
        
        void (*mFunctionCalledAfterUpdate)(Status*) = NULL;
        void (*mFunctionCalledAfterUpdate2)(void*, Status*) = NULL;
        void* mUserData;
        
        bool accelerationIsUpdated = false;
        bool attitudeIsUpdated = false;
        //bool beaconsIsUpdated = false;
        
        CleansingBeaconFilter cleansingBeaconFilter;
        
    public:
        
        Impl() : status(new Status()),
                mBeaconFilter(new BaseBeaconFilter())
        { }
        
        ~Impl(){}
        
        void initializeStatusIfZero(){
            if(status->states()->size()==0) {
                initializeStatus();
            }
        }
        
        void putAcceleration(const Acceleration acceleration){
            initializeStatusIfZero();

            mPedometer->putAcceleration(acceleration);
            accelerationIsUpdated = mPedometer->isUpdated();
            
            // TODO (Tentative implementation)
            if(accelerationIsUpdated && attitudeIsUpdated){
                predictMotionState(acceleration.timestamp());
            }
            
        }
        
        void putAttitude(const Attitude attitude){
            initializeStatusIfZero();
            mOrientationmeter->putAttitude(attitude);
            attitudeIsUpdated = mOrientationmeter->isUpdated();
            
            processResetStatus();
        }
        
        void predictMotionState(long timestamp){
            initializeStatusIfZero();
            
            if(previousTimestampMotion==0) previousTimestampMotion = timestamp;
            
            PoseRandomWalkerInput input;
            input.timestamp(timestamp);
            input.previousTimestamp(previousTimestampMotion);
            
            std::shared_ptr<States> states = status->states();
            States* statesPredicted = new States(mRandomWalker->predict(*states.get(), input));
            status->states(statesPredicted);
            
            if(mOptVerbose){
                std::cout << "prediction at t=" << timestamp << std::endl;
            }
            
            callback(status.get());
            
            previousTimestampMotion = timestamp;
        }
        
        void logStates(const States& states, const std::string& filename){
            std::stringstream ss;
            for(State s: states){
                ss << s << std::endl;
            }
            if(DataLogger::getInstance()){
                DataLogger::getInstance()->log(filename, ss.str());
            }
        }
        
        void doFiltering(const Beacons& beacons){
            if(beacons.size()==0){
                return;
            }
            
            long timestamp = beacons.timestamp();
            std::shared_ptr<States> states = status->states();
            
            // Logging before likelihood computation
            logStates(*states, "before_likelihood_states_"+std::to_string(timestamp)+".csv");
            
            //std::vector<double> vLogLLs = mObservationModel->computeLogLikelihood(*states, beacons);
            std::vector<std::vector<double>> vLogLLsAndMDists = mObservationModel->computeLogLikelihoodRelatedValues(*states, beacons);
            std::vector<double> vLogLLs(states->size());
            std::vector<double> mDists(states->size());
            for(int i=0; i<states->size(); i++){
                vLogLLs[i] = vLogLLsAndMDists.at(i).at(0);
                mDists[i] = vLogLLsAndMDists.at(i).at(1);
            }
            
            vLogLLs = weakenLogLikelihoods(vLogLLs, mAlphaWeaken);
            // Set negative log-likelihoods
            for(int i=0; i<vLogLLs.size(); i++){
                State& s = states->at(i);
                s.negativeLogLikelihood(-vLogLLs.at(i));
                s.mahalanobisDistance(mDists.at(i));
            }
            
            std::vector<double> weights = ArrayUtils::computeWeightsFromLogLikelihood(vLogLLs);
            double sumWeights = 0;
            // Multiply loglikelihood-based weights and particle weights.
            for(int i=0; i<weights.size(); i++){
                weights[i] = weights[i] * (states->at(i).weight());
                sumWeights += weights[i];
            }
            assert(sumWeights>0);
            // Renormalized
            for(int i=0; i<weights.size(); i++){
                weights[i] = weights[i]/sumWeights;
                states->at(i).weight(weights[i]);
            }
            
            // Logging after likelihood computation
            logStates(*states, "after_likelihood_states_"+std::to_string(timestamp)+".csv");
            
            States* statesNew = mResampler->resample(*states, &weights[0]);
            // Assign equal weights after resampling
            for(int i=0; i<weights.size(); i++){
                double weight = 1.0/(weights.size());
                statesNew->at(i).weight(weight);
            }
            status->states(statesNew);
            if(mOptVerbose){
                std::cout << "resampling at t=" << beacons.timestamp() << std::endl;
            }
            
            // Logging after resampling
            logStates(*statesNew, "resampled_states_"+std::to_string(timestamp)+".csv");
        }
        
        Beacons filterBeacons(const Beacons& beacons){
            size_t nBefore = beacons.size();
            const Beacons& beaconsCleansed = cleansingBeaconFilter.filter(beacons);
            Beacons beaconsFiltered = mBeaconFilter? mBeaconFilter->filter(beaconsCleansed) : beaconsCleansed;
            size_t nAfter = beaconsFiltered.size();
            if(mOptVerbose){
                if(nAfter!=nBefore){
                    std::cout << "BeaconFilter #beacons "<< nBefore << ">>" << nAfter <<std::endl;
                }
            }
            return beaconsFiltered;
        }
        
        void putBeacon(const Beacons & beacons){
            initializeStatusIfZero();
            
            const Beacons& beaconsFiltered = filterBeacons(beacons);
            if(beaconsFiltered.size()>0){
                if(checkIfDoFiltering()){
                    doFiltering(beaconsFiltered);
                }else{
                    if(mOptVerbose){
                        std::cout<<"resampling step was not applied."<<std::endl;
                    }
                }
            }
            callback(status.get());
        };
        
        static std::vector<double> weakenLogLikelihoods(std::vector<double> logLikelihoods, double alphaWeaken){
            size_t n = logLikelihoods.size();
            std::vector<double> weakenedLogLLs(n);
            for(int i=0; i<n; i++){
                weakenedLogLLs[i] = alphaWeaken*logLikelihoods[i];
            }
            return weakenedLogLLs;
        }
        
        void reset(){
            previousTimestampMotion = 0;
        }
        
        void initializeStatus(){
            this->reset();
            mPedometer->reset();
            mOrientationmeter->reset();
            States* states = new States(mStatusInitializer->initializeStates(mNumStates));
            updateStatus(states);
        }
        
        void updateStatus(States* states){
            Status *st = new Status();
            st->states(states);
            status.reset(st);
        }
        
        void updateHandler(void (*functionCalledAfterUpdate)(Status*)){
            mFunctionCalledAfterUpdate = functionCalledAfterUpdate;
        }
        
        void updateHandler(void (*functionCalledAfterUpdate)(void*, Status*), void* inUserData){
            mFunctionCalledAfterUpdate2 = functionCalledAfterUpdate;
            mUserData = inUserData;
        }
        
        Status* getStatus(){
            return status.get();
        };
        
        bool resetStatus(){
            initializeStatus();
            return true;
        }
        
        void callback(Status* status){
            if(mFunctionCalledAfterUpdate!=NULL){
                mFunctionCalledAfterUpdate(status);
            }
            if(mFunctionCalledAfterUpdate2!=NULL){
                mFunctionCalledAfterUpdate2(mUserData, status);
            }
        }
        
        bool resetStatus(Pose pose){
            bool orientationWasUpdated = mOrientationmeter->isUpdated();
            if(orientationWasUpdated){
                std::cout << "Orientation is updated. Reset succeeded." << std::endl;
                double orientationMeasured = mOrientationmeter->getYaw();
                States* states = new States(mStatusInitializer->resetStates(mNumStates, pose, orientationMeasured));
                status->states(states);
                callback(status.get());
                return true;
            }else{
                std::cout << "Orientation has not been updated. Reset input is cached to be processed later." << std::endl;
                std::function<void()> func = [this,pose](){
                    return resetStatus(pose);
                };
                functionsForReset.push(func);
                return false;
            }
        }
        
        bool resetStatus(Pose meanPose, Pose stdevPose){
            bool orientationWasUpdated = mOrientationmeter->isUpdated();
            if(orientationWasUpdated){
                std::cout << "Orientation is updated. Reset succeeded." << std::endl;
                double orientationMeasured = mOrientationmeter->getYaw();
                States* states = new States(mStatusInitializer->resetStates(mNumStates, meanPose, stdevPose, orientationMeasured));
                status->states(states);
                callback(status.get());
                return true;
            }else{
                std::cout << "Orientation has not been updated. Reset input is cached to be processed later." << std::endl;
                std::function<void()> func = [this,meanPose,stdevPose](){
                    return resetStatus(meanPose, stdevPose);
                };
                functionsForReset.push(func);
                return false;
            }
        }
        
        bool refineStatus(const Beacons& beacons){
            // TODO
            initializeStatusIfZero();
            const Beacons& beaconsFiltered = filterBeacons(beacons);
            if(beaconsFiltered.size()>0){
                if(checkIfDoFiltering()){
                    doFiltering(beaconsFiltered);
                }
            }
            std::shared_ptr<States> statesTmp = status->states();
            std::vector<Location> locations(statesTmp->begin(), statesTmp->end());
            States* statesNew = new States(mStatusInitializer->initializeStatesFromLocations(locations));
            status->states(statesNew);
            
            return false;
        }
        
        void processResetStatus(){
            if(functionsForReset.size()>0){
                bool orientationWasUpdated = mOrientationmeter->isUpdated();
                if(orientationWasUpdated){
                    std::function<void()> func = functionsForReset.back();
                    func();
                    functionsForReset.pop();
                    std::cout << "Stored reset request was processed." << std::endl;
                }
            }
        }
        
        bool checkIfDoFiltering() const{
            auto states = status->states();
            double variance2D = computeStates2DVariance(*states);
            double variance2DLowerBound = std::pow(mLocStdevLB.x(), 2)*std::pow(mLocStdevLB.y(),2);
            if(mOptVerbose){
                std::cout<<"variance2D="<<variance2D<<", "<<"variance2DLowerBound="<<variance2DLowerBound<<std::endl;
            }
            if(variance2D<variance2DLowerBound){
                return false;
            }else{
                return true;
            }
        }
        
        double computeStates2DVariance(const States& states) const{
            auto meanLoc = Location::mean(states);
            double varx = 0, vary = 0, covxy = 0; // = yx
            size_t n = states.size();
            for(int i=0; i<n; i++){
                const State& s = states.at(i);
                double dx = s.x() - meanLoc.x();
                double dy = s.y() - meanLoc.y();
                varx += dx*dx;
                vary += dy*dy;
                covxy += dx*dy;
            }
            varx/=n;
            vary/=n;
            covxy/=n;
            double det = varx*vary - covxy*covxy;
            return det;
        }
        
        void optVerbose(bool optVerbose){
            mOptVerbose = optVerbose;
        }
        
        void numStates(int numStates){
            mNumStates = numStates;
        }
        
        void alphaWeaken(double alphaWeaken){
            mAlphaWeaken = alphaWeaken;
        }
        
        void locationStandardDeviationLowerBound(Location loc){
            mLocStdevLB = loc;
        }
        
        void pedometer(std::shared_ptr<Pedometer> pedometer){
            mPedometer=pedometer;
        }
        
        void orientationmeter(std::shared_ptr<OrientationMeter> orientationMeter){
            mOrientationmeter = orientationMeter;
        }
        
        void systemModel(std::shared_ptr<SystemModel<State, PoseRandomWalkerInput>> randomWalker){
            mRandomWalker = randomWalker;
        }
        
        void observationModel(std::shared_ptr<ObservationModel<State, Beacons>> observationModel){
            mObservationModel = observationModel;
        }
        
        void resampler(std::shared_ptr<Resampler<State>> resampler){
            mResampler = resampler;
        }
        
        void statusInitializer(std::shared_ptr<StatusInitializer> statusInitializer){
            mStatusInitializer = statusInitializer;
        }
        
        void beaconFilter(std::shared_ptr<BeaconFilter> beaconFilter){
            mBeaconFilter = beaconFilter;
        }
        
    };
    
    
    // Delegation to StreamParticleFilter::Impl class
    
    StreamParticleFilter::StreamParticleFilter() : impl(new StreamParticleFilter::Impl()) {}
    
    StreamParticleFilter::~StreamParticleFilter(){}
    
    StreamParticleFilter& StreamParticleFilter::putAcceleration(const Acceleration acceleration){
        impl->putAcceleration(acceleration);
        return *this;
    }
    
    StreamParticleFilter& StreamParticleFilter::putAttitude(const Attitude attitude) {
        impl->putAttitude(attitude);
        return *this;
    }
    
    StreamParticleFilter& StreamParticleFilter::putBeacons(const Beacons beacons) {
        impl->putBeacon(beacons);
        return *this;
    }
    
    StreamParticleFilter& StreamParticleFilter::updateHandler(void (*functionCalledAfterUpdate)(Status*)) {
        impl->updateHandler(functionCalledAfterUpdate);
        return *this;
    }
    
    StreamParticleFilter& StreamParticleFilter::updateHandler(void (*functionCalledAfterUpdate)(void*, Status*), void* inUserData){
        impl->updateHandler(functionCalledAfterUpdate, inUserData);
        return *this;
    };
    
    
    Status* StreamParticleFilter::getStatus() {
        return impl->getStatus();
    }
    
    
    bool StreamParticleFilter::resetStatus(){
        return impl->resetStatus();
    }
    
    bool StreamParticleFilter::resetStatus(Pose pose){
        return impl->resetStatus(pose);
    }
    
    bool StreamParticleFilter::resetStatus(Pose meanPose, Pose stdevPose){
        return impl->resetStatus(meanPose, stdevPose);
    }
    
    bool StreamParticleFilter::refineStatus(const Beacons& beacons){
        return impl->refineStatus(beacons);
    }
    
    StreamParticleFilter& StreamParticleFilter::optVerbose(bool optVerbose){
        impl->optVerbose(optVerbose);
        return *this;
    }
    
    StreamParticleFilter& StreamParticleFilter::numStates(int numStates){
        impl->numStates(numStates);
        return *this;
    }
    
    StreamParticleFilter& StreamParticleFilter::alphaWeaken(double alphaWeaken){
        impl->alphaWeaken(alphaWeaken);
        return *this;
    }
    
    StreamParticleFilter& StreamParticleFilter::locationStandardDeviationLowerBound(loc::Location loc){
        impl->locationStandardDeviationLowerBound(loc);
        return *this;
    }
    
    
    StreamParticleFilter& StreamParticleFilter::pedometer(std::shared_ptr<Pedometer> pedometer){
        impl->pedometer(pedometer);
        return *this;
    }
    StreamParticleFilter& StreamParticleFilter::orientationMeter(std::shared_ptr<OrientationMeter> orientationMeter){
        impl->orientationmeter(orientationMeter);
        return *this;
    }
    
    StreamParticleFilter& StreamParticleFilter::systemModel(std::shared_ptr<SystemModel<State, PoseRandomWalkerInput>> randomWalker){
        impl->systemModel(randomWalker);
        return *this;
    }
    
    StreamParticleFilter& StreamParticleFilter::observationModel(std::shared_ptr<ObservationModel<State, Beacons>> observationModel){
        impl->observationModel(observationModel);
        return *this;
    }
    
    StreamParticleFilter& StreamParticleFilter::resampler(std::shared_ptr<Resampler<State>> resampler){
        impl->resampler(resampler);
        return *this;
    }
    
    StreamParticleFilter& StreamParticleFilter::statusInitializer(std::shared_ptr<StatusInitializer> statusInitializer){
        impl->statusInitializer(statusInitializer);
        return *this;
    }
    
    StreamParticleFilter& StreamParticleFilter::beaconFilter(std::shared_ptr<BeaconFilter> beaconFilter){
        impl->beaconFilter(beaconFilter);
        return *this;
    }
    
}