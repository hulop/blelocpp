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

#include "LocException.hpp"

namespace loc{
    
    // Parameter class implementation
    double StreamParticleFilter::FloorTransitionParameters::heightChangedCriterion() const{ return heightChangedCriterion_;
    }
    
    double StreamParticleFilter::FloorTransitionParameters::weightTransitionArea() const{
        return weightTransitionArea_;
    }
    
    StreamParticleFilter::FloorTransitionParameters& StreamParticleFilter::FloorTransitionParameters::heightChangedCriterion(double heightChanged){
        heightChangedCriterion_=heightChanged;
        return *this;
    }
    
    StreamParticleFilter::FloorTransitionParameters& StreamParticleFilter::FloorTransitionParameters::weightTransitionArea(double weight){
        weightTransitionArea_=weight;
        return *this;
    }
    
    double StreamParticleFilter::FloorTransitionParameters::mixtureProbaTransArea() const{ return mixtureProbaTransArea_;
    }
    
    double StreamParticleFilter::FloorTransitionParameters::rejectDistance() const {
        return rejectDistance_;
    }
    
    StreamParticleFilter::FloorTransitionParameters& StreamParticleFilter::FloorTransitionParameters::mixtureProbaTransArea(double proba){
        mixtureProbaTransArea_=proba;
        return *this;
    }
    
    StreamParticleFilter::FloorTransitionParameters& StreamParticleFilter::FloorTransitionParameters::rejectDistance(double dist){
        rejectDistance_=dist;
        return *this;
    }
    
    
    // Helper class implementations
    
    class FloorUpdater{
        
    public:
        FloorUpdateMode mode;
        ObservationModel<State, Beacons>::Ptr mObsModel;
        DataStore::Ptr mDataStore;
        RandomGenerator::Ptr randomGenerator;
        bool mVerbose = false;
        
        void floorUpdate(States& states, const Beacons& beacons){
            const BLEBeacons& bleBeacons = mDataStore->getBLEBeacons();
            auto knownBeacons = BLEBeacon::filter(beacons, bleBeacons);
            
            if(mode==COUNT){
                floorUpdateSimple(states, knownBeacons);
            }else if(mode==WEIGHT){
                floorUpdateUsingObservationModel(states,knownBeacons);
            }else{
                BOOST_THROW_EXCEPTION(LocException("Unknown floor update mode."));
            }
        }
        
    protected:
        /**
         * param beacons a vector of input beacons
         * param bleBeacons a vector of BLE beacons
         * return Map of (floor, count)
         **/
        std::map<int, int> countFloors(const Beacons& beacons, const BLEBeacons& bleBeacons){
            std::map<int, int> obsFloors;
            // Add floors
            for(const auto& b: beacons){
                auto ble = BLEBeacon::find(b, bleBeacons);
                int floor = std::round(ble.floor());
                if(obsFloors.count(floor) == 0){
                    obsFloors[floor] = 1;
                }else{
                    obsFloors[floor] +=1 ;
                }
            }
            return obsFloors;
        }
        
        void floorUpdateSimple(States& states, const Beacons& beacons){
            if(beacons.size()==0){
                return;
            }
            if(!mDataStore){
                return;
            }
            const BLEBeacons& bleBeacons = mDataStore->getBLEBeacons();
            const Building& building = mDataStore->getBuilding();
            // Add floors
            std::map<int, int> obsFloors = countFloors(beacons, bleBeacons);

            // Find floor from observed beacons.
            int repFloor = 0;
            int maxcount = -1;
            for(auto iter = obsFloors.begin(); iter!=obsFloors.end(); iter++){
                int floor = (*iter).first;
                int count = (*iter).second;
                if(count > maxcount){
                    repFloor = floor;
                    maxcount = count;
                }
            }
            // Update floor when repFloor is different from state.floor
            for(auto& s: states){
                int floor = std::round(s.floor());
                if(obsFloors.count(floor) == 0){
                    State sTmp(s);
                    sTmp.floor(repFloor);
                    if(building.isMovable(sTmp)){
                        s.floor(repFloor);
                    }
                }
            }
        }
        
        void floorUpdateUsingObservationModel(States& states, const Beacons& beacons){
            if(beacons.size()==0){
                return;
            }
            if(!mDataStore){
                return;
            }
            
            const BLEBeacons& bleBeacons = mDataStore->getBLEBeacons();
            const Building& building = mDataStore->getBuilding();
            
            // Add floors
            std::map<int, int> obsFloors = countFloors(beacons, bleBeacons);

            State meanState = State::weightedMean(states);
            std::vector<int> floors;
            States statesTmp;
            for(auto iter = obsFloors.begin(); iter!=obsFloors.end(); iter++){
                int floor = (*iter).first;
                State sTmp(meanState);
                sTmp.floor(floor);
                floors.push_back(floor);
                statesTmp.push_back(sTmp);
            }
            assert(statesTmp.size() == floors.size());
            auto logLLs = mObsModel->computeLogLikelihood(statesTmp, beacons);
            
            //if(true){
            //    double avgLogLL = std::accumulate(logLLs.begin(), logLLs.end(), 0.0)/(logLLs.size());
            //    std::cout << "#beacons=" << beacons.size() << ",avgLogLL = " << avgLogLL << std::endl;
            //}
            
            std::vector<double> weights = ArrayUtils::computeWeightsFromLogLikelihood(logLLs);
            
            // generate floors using weights and random numbers
            std::vector<int> floorsGenerated;
            for(int i = 0; i<states.size(); i++){
                double d = randomGenerator->nextDouble();
                double sumw = 0;
                int floorGen = std::numeric_limits<int>::max();
                for(int i=0; i<weights.size(); i++){
                    sumw+=weights.at(i);
                    if(d<=sumw){
                        floorGen = floors.at(i);
                        break;
                    }
                    if(i==weights.size()-1){
                        floorGen = floors.at(i);
                    }
                }
                assert(floorGen != std::numeric_limits<int>::max());
                floorsGenerated.push_back(floorGen);
            }
            
            // update floors
            std::vector<int> floorsWritten(states.size());
            for(int i=0; i<states.size(); i++){
                auto&s = states.at(i);
                int floor = std::round(s.floor());
                int floorGen = floorsGenerated.at(i);
                floorsWritten.at(i) = floor;
                if(floor!=floorGen){
                    State sTmp(s);
                    sTmp.floor(floorGen);
                    if(building.isMovable(sTmp)){
                        s.floor(floorGen);
                        floorsWritten.at(i) = floorGen;
                    }
                }
            }
            
            size_t fsize = 0;
            if(mVerbose){
                std::stringstream ss;
                ss << "(floor,weights,countGenerated,countWritten)=";
                for(int i=0; i<weights.size(); i++){
                    int floor = floors.at(i);
                    auto countGen = std::count(floorsGenerated.begin(), floorsGenerated.end(), floor);
                    auto countWri = std::count(floorsWritten.begin(), floorsWritten.end(), floor);
                    ss << "("<<floor<<","<<weights.at(i)<<"," << countGen << "," <<countWri <<")," ;
                }
                std::cout << ss.str() << std::endl;
                
                if(floors.size()>=2){
                    fsize = floors.size();
                }
            }
        }
    };
    

    class StreamParticleFilter::Impl{

    private:
        bool mOptVerbose = false;
        int mNumStates = 1000; //Defalt value
        double mAlphaWeaken = 1.0;
        int resetWaitingTimeMS = 100; // milliseconds
        long previousTimestampMotion = 0;
        long timestampIntervalLimit = 2000; // 2.0[s]
        
        double mEssThreshold = 10000; //effective sampling size (typically nNumStates/2. nNumState<=essThreshold for frequent resampling.)
        Location mLocStdevLB;
        
        bool mEnablesFloorUpdate = true;
        
        MixtureParameters mMixParams;
        FloorTransitionParameters::Ptr mFloorTransParams = std::make_shared<FloorTransitionParameters>();
        
        long previousTimestampMonitoring = 0;;
        LocationStatusMonitorParameters::Ptr mLocStatusMonitorParams = std::make_shared<LocationStatusMonitorParameters>();
        
        //std::queue<Pose> posesForReset;
        std::queue<std::function<void()>> functionsForReset;

        std::shared_ptr<Status> status;

        std::shared_ptr<Pedometer> mPedometer;
        std::shared_ptr<OrientationMeter> mOrientationmeter;
        std::shared_ptr<AltitudeManager> mAltitudeManager;

        std::shared_ptr<SystemModel<State, SystemModelInput>> mRandomWalker;

        std::shared_ptr<ObservationModel<State, Beacons>> mObservationModel;
        std::shared_ptr<Resampler<State>> mResampler;
        std::shared_ptr<StatusInitializer> mStatusInitializer;
        std::shared_ptr<BeaconFilter> mBeaconFilter;

        std::shared_ptr<ObservationDependentInitializer<State, Beacons>> mMetro;
        std::shared_ptr<PosteriorResampler<State>> mPostResampler;
        std::shared_ptr<RandomGenerator> mRand;
        
        DataStore::Ptr mDataStore;
        
        std::shared_ptr<FloorUpdater> mFloorUpdater;
        FloorUpdateMode mFloorUpdateMode = WEIGHT;
        bool mFiltersBeaconFloorAtReset = false;
        
        void (*mFunctionCalledAfterUpdate)(Status*) = NULL;
        void (*mFunctionCalledAfterUpdate2)(void*, Status*) = NULL;
        void* mUserData;

        bool accelerationIsUpdated = false;
        bool attitudeIsUpdated = false;
        //bool beaconsIsUpdated = false;

        CleansingBeaconFilter cleansingBeaconFilter;

    public:

        Impl() : status(new Status()),
                mBeaconFilter(new BaseBeaconFilter()),
                mRand(new RandomGenerator())
        { }

        ~Impl(){}

        void initializeStatusIfZero(){
            if(status->states()->size()==0) {
                initializeStatus();
            }
        }

        void putAcceleration(const Acceleration acceleration){
            initializeStatusIfZero();
            status->step(Status::OTHER);
            
            mPedometer->putAcceleration(acceleration);
            accelerationIsUpdated = mPedometer->isUpdated();

            // TODO (Tentative implementation)
            if(accelerationIsUpdated && attitudeIsUpdated){
                predictMotionState(acceleration.timestamp());
            }

        }

        void putAttitude(const Attitude attitude){
            initializeStatusIfZero();
            status->step(Status::OTHER);
            
            mOrientationmeter->putAttitude(attitude);
            attitudeIsUpdated = mOrientationmeter->isUpdated();

            processResetStatus();
        }
        
        void predictMotionState(long timestamp){
            initializeStatusIfZero();

            if(previousTimestampMotion==0){
                previousTimestampMotion = timestamp;
                return;
            }

            SystemModelInput input;
            input.timestamp(timestamp);
            input.previousTimestamp(previousTimestampMotion);

            std::shared_ptr<States> states = status->states();
            
            bool timestampIntervalIsValid = input.timestamp() - input.previousTimestamp() < timestampIntervalLimit;
            
            if(timestampIntervalIsValid){
                StatesPtr statesPredicted(new States(mRandomWalker->predict(*states.get(), input)));
                status->states(statesPredicted, Status::PREDICTION);
            }else{
                std::cout << "Interval between two timestamps is too large. The input at timestamp=" << timestamp << " was not used." << std::endl;
            }
            
            status->timestamp(timestamp);

            if(mOptVerbose){
                std::cout << "prediction at t=" << timestamp << std::endl;
            }

            if(status->step()==Status::PREDICTION){
                callback(status.get());
            }
            
            previousTimestampMotion = timestamp;
        }
        
        void putAltimeter(const Altimeter altimeter){
            if(mAltitudeManager){
                mAltitudeManager->putAltimeter(altimeter);
                
                // Update states with the altimeter manager.
                long ts = altimeter.timestamp();
                std::shared_ptr<States> states = status->states();
                auto statesNew = this->predictFloorTransState(states);
                status->timestamp(ts);
                status->states(statesNew);
                callback(status.get());
            }
        }
        
        StatesPtr predictFloorTransState(const StatesPtr& states){
            auto heightChanged = mAltitudeManager->heightChange();
            const auto& building = mDataStore->getBuilding();
            
            auto statesNew = StatesPtr(new States(*states));
            
            if(heightChanged > mFloorTransParams->heightChangedCriterion()){
                // multiply weight by coeff in transition area.
                size_t nTrans = 0;
                size_t n = statesNew->size();
                double coeff = mFloorTransParams->weightTransitionArea();
                double sumWeights = 0.0;
                for(auto& s: *statesNew){
                    if(building.isTransitionArea(s)){
                        s.weight(s.weight() * coeff);
                        nTrans++;
                    }
                    sumWeights += s.weight();
                }
                if(sumWeights<=0){
                    LocException ex("sum(weights) <= 0");
                    BOOST_THROW_EXCEPTION(ex);
                }
                // normalize weights
                for(auto&s: *statesNew){
                    double w = s.weight()/sumWeights;
                    s.weight(w);
                }
                
                // mix for floor transition area
                double ratioTrans = static_cast<double>(nTrans)/static_cast<double>(n);
                int nMixed = 0;
                if(ratioTrans < mFloorTransParams->mixtureProbaTransArea()){
                    double ratioResid = mFloorTransParams->mixtureProbaTransArea() - ratioTrans;
                    for(auto& s: *statesNew){
                        if(building.isTransitionArea(s)){
                            continue;
                        }
                        double d = mRand->nextDouble();
                        if( d < ratioResid){
                            const auto& floorMap = building.getFloorAt(s);
                            auto locsTA = floorMap.findClosestTransitionAreaLocations(s);
                            if(locsTA.size()==0){
                                continue;
                            }
                            auto locTA = locsTA.at(0);
                            if( mFloorTransParams->rejectDistance() <= Location::distance(s, locTA)){
                                continue;
                            }else{
                                s.copyLocation(locTA);
                                nMixed++;
                            }
                        }
                    }
                }
                
                if(mOptVerbose){
                    std::stringstream ss;
                    ss << "AltimeterManager detected height change. " << nTrans << "/" << n << " states are " << coeff << "x-weighted.";
                    if(nMixed!=0){
                        ss << "(" << nMixed << " states in floor transition area were mixed.)";
                    }
                    std::cout << ss.str() << std::endl;
                }
            }
            return statesNew;
        }

        void logStates(const States& states, const std::string& filename){
            if(DataLogger::getInstance()){
                DataLogger::getInstance()->log(filename, DataUtils::statesToCSV(states));
            }
        }
        
        States generateStatesForMix(int nGen, const Beacons& beacons, const MixtureParameters& mixParams,
                                    std::vector<State>& allGeneratedStates, std::vector<double>& allGeneratedStatesLogLLs
                                    ){
            // Generate states
            int burnInLight = mixParams.burnInQuick;
            States statesGen;
            if(mMetro){
                mMetro->input(beacons);
                mMetro->startBurnIn(burnInLight);
                statesGen = mMetro->sampling(nGen);
                //mMetro->print();
                auto allStates =  mMetro->getAllStates();
                auto allLogLLs = mMetro->getAllLogLLs();
                allGeneratedStates = allStates;
                allGeneratedStatesLogLLs = allLogLLs;
                //double avgLogLL = std::accumulate(allLogLLs.begin(), allLogLLs.end(), 0.0)/allLogLLs.size();
                //std::cout << "avgLogLL=" << avgLogLL << std::endl;
            }else{
                statesGen = mStatusInitializer->resetStates(nGen, beacons);
            }
            return statesGen;
        }
        
        States mixStates(const States& states, const Beacons& beacons, const MixtureParameters& mixParams, bool evaluatesLLs,
                       std::vector<State>& allGeneratedStates, std::vector<double>& allGeneratedStatesLogLLs
                       ){
            if( beacons.size() < mixParams.nBeaconsMinimum){
                return states;
            }
            size_t nStates = states.size();
            std::vector<int> indices;
            // Select particles that will be removed randomly.
            for(int i=0; i<nStates; i++){
                double u = mRand->nextDouble();
                if(u< mixParams.mixtureProbability){
                    indices.push_back(i);
                }
            }
            int nGen = static_cast<int>(indices.size());
            
            //// do burn-in even if nGen==0 to evaluate likelihood
            if(nGen==0 && !evaluatesLLs){
                return states;
            }
            
            States statesGen = generateStatesForMix(nGen, beacons, mixParams, allGeneratedStates, allGeneratedStatesLogLLs);
            
            States statesMixed(states);
            //Location locMean = Location::mean(states);
            // Copy location of generated states to the existing states.
            for(int i=0; i<nGen; i++){
                auto& st = statesGen.at(i);
                //double p = computeStateAcceptProbability(locMean, st); //compate mean state and new state.
                double p = computeStateAcceptProbability(states, st); //compare all states and new state.
                if(mRand->nextDouble() < p ){
                    int idx = indices.at(i);
                    statesMixed.at(idx).copyLocation(st);
                }
            }
            
            return statesMixed;
        }
        
        double computeStateAcceptProbability(const Location& locMean, const Location& locNew){
            double floorDiff = Location::floorDifference(locMean, locNew);
            double dist = Location::distance(locMean, locNew);
            if(mMixParams.rejectFloorDifference() < floorDiff){
                return 1;
            }
            if(mMixParams.rejectDistance < dist){
                return 1;
            }
            return 0;
        }
        
        double computeStateAcceptProbability(const States& states, const Location& locNew){
            size_t n = states.size();
            double sumDist = 0;
            double sumIsFloorDifferent = 0;
            for(const auto& s: states){
                sumDist += Location::distance(s, locNew);
                double isFloorDifferent = Location::floorDifference(s, locNew)>0.5 ? 1 : 0;
                sumIsFloorDifferent += isFloorDifferent;
            }
            double meanDist = sumDist/n;
            double meanIsFloorDifferent = sumIsFloorDifferent/n;
            
            if(mMixParams.rejectFloorDifference() < meanIsFloorDifferent){
                return 1;
            }
            if(mMixParams.rejectDistance < meanDist){
                return 1;
            }
            return 0;
        }
        
        void doFiltering(const Beacons& beacons){
            this->updateStatusByBeacons(beacons, true, true);
        }
        
        void updateStatusByBeacons(const Beacons& beacons, const bool& doesFiltering, const bool& monitorsStatus){

            if(beacons.size()==0){
                return;
            }

            long timestamp = beacons.timestamp();
            
            status->timestamp(timestamp);
            std::shared_ptr<States> states = status->states();
            
            bool passedMonitoringInterval = false;
            if(timestamp - previousTimestampMonitoring > mLocStatusMonitorParams->monitorIntervalMS() ){
                passedMonitoringInterval = true;
                previousTimestampMonitoring = timestamp;
            }
            
            // Compute states mixed with states generated from observations
            std::vector<State> allMixStates;
            std::vector<double> allMixLogLLs;
            States statesMixed;
            if(passedMonitoringInterval || mMixParams.mixtureProbability>0){
                statesMixed = mixStates(*states, beacons, mMixParams, passedMonitoringInterval, allMixStates, allMixLogLLs);
            }else{
                statesMixed = *states;
            }
            if(doesFiltering){
                // Logging before weights updated
                logStates(*states, "before_likelihood_states_"+std::to_string(timestamp)+".csv");
                // Copy mixed states when apply filtering
                *states = statesMixed;
            }
            
            // Compute log likelihood
            std::vector<std::vector<double>> vLogLLsAndMDists = mObservationModel->computeLogLikelihoodRelatedValues(*states, beacons);
            std::vector<double> vLogLLs(states->size());
            std::vector<double> mDists(states->size());
            for(int i=0; i<states->size(); i++){
                vLogLLs[i] = vLogLLsAndMDists.at(i).at(0);
                mDists[i] = vLogLLsAndMDists.at(i).at(1);
            }
            
            if(monitorsStatus){
                // Update locationStatus by comparing likelihoods between states and one-shot states
                
                double avgCurrentLogLL = std::accumulate(vLogLLs.begin(), vLogLLs.end(), 0.0)/vLogLLs.size();
                double avgMixLogLL = std::accumulate(allMixLogLLs.begin(), allMixLogLLs.end(), 0.0)/allMixLogLLs.size();
                
                if(!isnan(avgMixLogLL)){
                    double maxCurrentLogLL = *std::max_element(vLogLLs.begin(), vLogLLs.end());
                    double maxMixLogLL = *std::max_element(allMixLogLLs.begin(), allMixLogLLs.end());
                    
                    double weightAvgLogLL = std::exp(avgCurrentLogLL)/(std::exp(avgCurrentLogLL)+std::exp(avgMixLogLL));
                    double weightMaxLogLL = std::exp(maxCurrentLogLL)/(std::exp(maxCurrentLogLL)+std::exp(maxMixLogLL));
                    double wTol = mLocStatusMonitorParams->minimumWeightStable();
                    
                    double weightInStates = weightMaxLogLL;
                    
                    auto locationStatus = status->locationStatus();
                    if(mOptVerbose){
                        std::cout << "locationStatus = " << Status::locationStatusToString(locationStatus) << std::endl;
                        std::cout << "Average logLikelihood (inStates,inMix)=(" << avgCurrentLogLL << "," << avgMixLogLL << "), weightAvgLogLL=" << weightAvgLogLL << ",wTol=" << wTol << std::endl;
                        std::cout << "Max logLikelihood (inStates,inMix)=(" << maxCurrentLogLL << "," << maxMixLogLL << "), weightMaxLogLL=" << weightMaxLogLL << ",wTol=" << wTol << std::endl;
                    }
                    if(locationStatus==Status::STABLE){
                        if(weightInStates < wTol){
                            locationStatus=Status::UNSTABLE;
                        }
                    }else if(locationStatus==Status::UNSTABLE){
                        if(weightInStates < wTol){
                            locationStatus=Status::UNKNOWN;
                        }else{
                            locationStatus=Status::STABLE;
                        }
                    }
                    status->locationStatus(locationStatus);
                }
            }
            
            if(doesFiltering){
                // Apply alpha-weaken
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
                if(sumWeights<=0){
                    LocException ex("sum(weights) <= 0");
                    for(auto logLL: vLogLLs){
                        if(logLL == 0){
                            ex << boost::error_info<struct error_info, std::string>("logLikelihood == 0. (Probably, input beacons are unknown.)");
                            break;
                        }
                    }
                    BOOST_THROW_EXCEPTION(ex);
                }
                // Renormalized
                for(int i=0; i<weights.size(); i++){
                    weights[i] = weights[i]/sumWeights;
                    states->at(i).weight(weights[i]);
                }
                
                // Logging after weights updated
                logStates(*states, "after_likelihood_states_"+std::to_string(timestamp)+".csv");
                
                // Resampling step
                double ess = computeESS(weights);
                if(mOptVerbose){
                    std::cout << "ESS=" << ess << std::endl;
                }
                StatesPtr statesNew;
                Status::Step step;
                
                if(ess<=mEssThreshold){
                    statesNew.reset(mResampler->resample(*states, &weights[0]));
                    // Assign equal weights after resampling
                    for(int i=0; i<weights.size(); i++){
                        double weight = 1.0/(weights.size());
                        statesNew->at(i).weight(weight);
                    }
                    step = Status::FILTERING_WITH_RESAMPLING;
                }else{
                    statesNew.reset(new States(*states));
                    step = Status::FILTERING_WITHOUT_RESAMPLING;
                }
                
                // Posterior-resampling
                if(mPostResampler){
                    *statesNew = mPostResampler->resample(*statesNew);
                }
                
                status->states(statesNew, step);
                if(mOptVerbose){
                    std::cout << "resampling at t=" << beacons.timestamp() << std::endl;
                }
                // Logging after resampling
                logStates(*statesNew, "resampled_states_"+std::to_string(timestamp)+".csv");
                
                // Notify registered instances of the update of particle fiter
                this->notifyObservationUpdated();
            }else{
                return;
            }
        }

        void notifyObservationUpdated(){
            mRandomWalker->notifyObservationUpdated();
        }
        
        double computeESS(std::vector<double> weights){
            double val = 0;
            for(double w : weights){
                val += w*w;
            }
            double ess = 1.0/val;
            return ess;
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

        Beacons filterBeaconsByStrongestBeaconFloor(const Beacons& beacons, const BLEBeacons& blebeacons){
            auto beaconsSorted = Beacon::sortByRssi(beacons);
            const auto& strongestBeacon = beaconsSorted.back();
            for(int i = 0; i<beaconsSorted.size(); i++){
                auto r1 = beaconsSorted.at(i).rssi();
                assert(r1<=strongestBeacon.rssi());
            }
            BLEBeacon strongestLoc = BLEBeacon::find(strongestBeacon, blebeacons);
            double floor_est = strongestLoc.floor();
            Beacons beaconsFloorEst(beacons);
            beaconsFloorEst.clear();
            assert(beaconsFloorEst.size()==0);
            for(const Beacon& b: beaconsSorted){
                BLEBeacon ble =  BLEBeacon::find(b, blebeacons);
                assert(b.major() == ble.major());
                assert(b.minor() == ble.minor());
                if(ble.floor() == floor_est){
                    beaconsFloorEst.push_back(b);
                }
            }
            std::cout << "floor_est=" << floor_est << ", #beacons " << beacons.size() << " -> " << beaconsFloorEst.size() << std::endl;
            return beaconsFloorEst;
        }
        
        bool checkTryFloorUpdate(){
            if(mEnablesFloorUpdate){
                if(mAltitudeManager){
                    auto heightChanged = mAltitudeManager->heightChange();
                    if(heightChanged > mFloorTransParams->heightChangedCriterion()){
                        return true;
                    }else{
                        return false;
                    }
                }else{
                    return true;
                }
            }else{
                return false;
            }
        }
        
        void putBeacons(const Beacons& beacons){
            initializeStatusIfZero();
            status->step(Status::OTHER);
            
            const Beacons& beaconsFiltered = filterBeacons(beacons);
            if(beaconsFiltered.size()>0){
                // Observation dependent floor update
                std::shared_ptr<States> states = status->states();
                bool tryFloorUpdate = false;
                if(mEnablesFloorUpdate){
                    if(!mFloorUpdater){
                        mFloorUpdater.reset(new FloorUpdater);
                        mFloorUpdater->mode = mFloorUpdateMode;
                        mFloorUpdater->mDataStore = mDataStore;
                        mFloorUpdater->mObsModel = mObservationModel;
                        mFloorUpdater->mVerbose = mOptVerbose;
                        mFloorUpdater->randomGenerator = mRand;
                    }
                    tryFloorUpdate = checkTryFloorUpdate();
                    if(tryFloorUpdate){
                        mFloorUpdater->floorUpdate(*states, beaconsFiltered);
                        status->states(states);// update states to compute rep values.
                    }
                }
                // filtering
                bool doesFiltering = checkIfDoFiltering(*states);
                bool monitorsStatus = true;
                
                if(doesFiltering){
                    updateStatusByBeacons(beaconsFiltered, doesFiltering, monitorsStatus);
                    assert( status->step()==Status::FILTERING_WITH_RESAMPLING
                           || status->step()==Status::FILTERING_WITHOUT_RESAMPLING );
                }else{
                    updateStatusByBeacons(beaconsFiltered, doesFiltering, monitorsStatus);
                    if(mOptVerbose){
                        std::cout<<"filtering step was not applied."<<std::endl;
                    }
                    status->step(Status::OBSERVATION_WITHOUT_FILTERING);
                }
                if(mEnablesFloorUpdate){
                    if(tryFloorUpdate){
                        status->wasFloorUpdated(true);
                    }
                }
            }else{
                status->step(Status::OBSERVATION_WITHOUT_FILTERING);
            }
            status->timestamp(beacons.timestamp());
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
            StatesPtr states(new States(mStatusInitializer->initializeStates(mNumStates)));
            updateStatus(states);
        }

        void updateStatus(StatesPtr states){
            Status *st = new Status();
            st->states(states, Status::OTHER);
            status.reset(st);
        }

        void updateHandler(void (*functionCalledAfterUpdate)(Status*)){
            mFunctionCalledAfterUpdate = functionCalledAfterUpdate;
        }

        void updateHandler(void (*functionCalledAfterUpdate)(void*, Status*), void* inUserData){
            mFunctionCalledAfterUpdate2 = functionCalledAfterUpdate;
            mUserData = inUserData;
        }
        
        void dataStore(DataStore::Ptr dataStore){
            mDataStore = dataStore;
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
                StatesPtr states(new States(mStatusInitializer->resetStates(mNumStates, pose, orientationMeasured)));
                status->states(states, Status::RESET);
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
                StatesPtr states(new States(mStatusInitializer->resetStates(mNumStates, meanPose, stdevPose, orientationMeasured)));
                status->states(states, Status::RESET);
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
        
        bool resetStatus(Pose meanPose, Pose stdevPose, double rateContami){
            bool orientationWasUpdated = mOrientationmeter->isUpdated();
            if(orientationWasUpdated){
                std::cout << "Orientation is updated. Reset(meanPose, stdevPose, rateContami) succeeded." << std::endl;
                double orientationMeasured = mOrientationmeter->getYaw();
                auto statesTmp = mStatusInitializer->resetStates(mNumStates, meanPose, stdevPose, orientationMeasured);
                for(auto& s: statesTmp){
                    double d = mRand->nextDouble();
                    if(d<rateContami){
                        double orientationBias = 2.0*M_PI*mRand->nextDouble();
                        double orientation = orientationMeasured - orientationBias;
                        s.orientationBias(orientationBias);
                        s.orientation(orientation);
                    }
                }
                StatesPtr states(new States(statesTmp));
                status->states(states, Status::RESET);
                callback(status.get());
                return true;
            }else{
                std::cout << "Orientation has not been updated. Reset input is cached to be processed later." << std::endl;
                std::function<void()> func = [this,meanPose,stdevPose, rateContami](){
                    return resetStatus(meanPose, stdevPose, rateContami);
                };
                functionsForReset.push(func);
                return false;
            }
        }

        bool resetStatus(const Beacons& beacons){
            initializeStatusIfZero();
            Beacons beaconsFiltered = filterBeacons(beacons);
            if(mDataStore && mFiltersBeaconFloorAtReset){
                const auto& blebeacons = mDataStore->getBLEBeacons();
                beaconsFiltered = filterBeaconsByStrongestBeaconFloor(beaconsFiltered, blebeacons);
            }
            std::stringstream ss;
            ss << "Status was initialized by ";
            if(beaconsFiltered.size() == 0){
                BOOST_THROW_EXCEPTION(LocException("beaconsFiltered.size==0 in resetStatus(beacons)."));
            }
            States statesTmp = sampleStatesByObservation(mNumStates, beaconsFiltered);
            StatesPtr statesNew(new States(statesTmp));
            status->states(statesNew, Status::RESET);
            status->timestamp(beacons.timestamp());
            if(mMetro){
                ss << "an ObservationDependentInitializer.";
            }else{
                ss << "a StatusInitializer.";
            }
            std::cout << ss.str() << std::endl;
            return false;
        }
        
        States sampleStatesByObservation(int n, const Beacons& beacons){
            const Beacons& beaconsFiltered = filterBeacons(beacons);
            States statesNew;
            if(mMetro){
                mMetro->input(beaconsFiltered);
                mMetro->startBurnIn();
                States states = mMetro->sampling(n);
                std::vector<Location> locations(states.begin(), states.end());
                statesNew = mStatusInitializer->initializeStatesFromLocations(locations);
                for(int i=0; i<states.size(); i++){
                    statesNew.at(i).rssiBias(states.at(i).rssiBias());
                }
            }else{
                statesNew = mStatusInitializer->resetStates(n, beaconsFiltered);
            }
            return statesNew;
        }
        
        bool resetStatus(const Location& location, const Beacons& beacons){
            initializeStatusIfZero();
            const Beacons& beaconsFiltered = filterBeacons(beacons);
            std::stringstream ss;
            ss << "Status was initialized by ";
            States statesTmp = sampleStatesByLocationAndObservation(mNumStates, location, beaconsFiltered);
            StatesPtr statesNew(new States(statesTmp));
            status->states(statesNew, Status::RESET);
            status->timestamp(beacons.timestamp());
            if(mMetro){
                ss << "an ObservationDependentInitializer.";
            }else{
                ss << "a StatusInitializer.";
            }
            std::cout << ss.str() << std::endl;
            return false;
        }
        
        States sampleStatesByLocationAndObservation(int n, const Location& location, const Beacons& beacons){
            const Beacons& beaconsFiltered = filterBeacons(beacons);
            States statesNew;
            if(mMetro){
                mMetro->input(beaconsFiltered);
                statesNew = mMetro->sampling(n, location);
                //States states = mMetro->sampling(n);
                //std::vector<Location> locations(states.begin(), states.end());
                //statesNew = mStatusInitializer->initializeStatesFromLocations(locations);
            }else{
                statesNew = mStatusInitializer->resetStates(n, beaconsFiltered);
            }
            return statesNew;
        }


        bool refineStatus(const Beacons& beacons){
            // TODO
            BOOST_THROW_EXCEPTION(LocException("unsupported method"));
            
            const Beacons& beaconsFiltered = filterBeacons(beacons);

            if(beaconsFiltered.size()>0){
                auto states = status->states();
                if(checkIfDoFiltering(*states)){
                    doFiltering(beaconsFiltered);
                }
            }
            std::shared_ptr<States> statesTmp = status->states();
            std::vector<Location> locations(statesTmp->begin(), statesTmp->end());
            StatesPtr statesNew(new States(mStatusInitializer->initializeStatesFromLocations(locations)));
            status->timestamp(beacons.timestamp());
            status->states(statesNew, Status::RESET);
            callback(status.get());
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

        bool checkIfDoFiltering(const States& states) const{
            double variance2DLowerBound = std::pow(mLocStdevLB.x(), 2)*std::pow(mLocStdevLB.y(),2);
            double stdZLB = mLocStdevLB.z();
            double stdFloorLB = mLocStdevLB.floor();
            
            double variance2D = Location::compute2DVariance(states);
            Location stdevLoc = Location::standardDeviation(states);
            
            if(mOptVerbose){
                std::cout<<"var2D="<<variance2D<<","<<"var2DLB="<<variance2DLowerBound
                << ",stdZ=" << stdevLoc.z() << ",stdZLB=" << stdZLB
                << ",stdFloor=" << stdevLoc.floor() << ",stdFloorLB=" << stdFloorLB
                << std::endl;
            }
            if(variance2D <= variance2DLowerBound
               && stdevLoc.z() <= stdZLB
               && stdevLoc.floor() <= stdFloorLB){
                return false;
            }else{
                return true;
            }
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
        
        void essThreshold(double essThreshold){
            mEssThreshold = essThreshold;
        }
        
        void mixtureParameters(MixtureParameters mixParams){
            mMixParams = mixParams;
        }

        void locationStandardDeviationLowerBound(Location loc){
            mLocStdevLB = loc;
        }
        
        void enablesFloorUpdate(bool enablesFloorUpdate){
            mEnablesFloorUpdate = enablesFloorUpdate;
        }

        void pedometer(std::shared_ptr<Pedometer> pedometer){
            mPedometer=pedometer;
        }

        void orientationmeter(std::shared_ptr<OrientationMeter> orientationMeter){
            mOrientationmeter = orientationMeter;
        }
        
        void altitudeManager(std::shared_ptr<AltitudeManager> altitudeManager){
            mAltitudeManager = altitudeManager;
        }

        void systemModel(std::shared_ptr<SystemModel<State, SystemModelInput>> randomWalker){
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

        void observationDependentInitializer(std::shared_ptr<ObservationDependentInitializer<State, Beacons>> metro){
            mMetro = metro;
        }
        
        void posteriorResampler(std::shared_ptr<PosteriorResampler<State>> postRes){
            mPostResampler = postRes;
        }
        
        void floorUpdateMode(FloorUpdateMode mode){
            mFloorUpdateMode = mode;
        }
        
        void floorTransitionParameters(FloorTransitionParameters::Ptr params){
            mFloorTransParams = params;
        }
        
        void locationStatusMonitorParameters(LocationStatusMonitorParameters::Ptr params){
            mLocStatusMonitorParams = params;
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
        impl->putBeacons(beacons);
        return *this;
    }
    
    StreamParticleFilter& StreamParticleFilter::putLocalHeading(const LocalHeading heading) {
        // Pass
        BOOST_THROW_EXCEPTION(LocException("not supported"));
        return *this;
    }

    StreamParticleFilter& StreamParticleFilter::putAltimeter(const Altimeter altimeter) {
        impl->putAltimeter(altimeter);
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
    
    bool StreamParticleFilter::resetStatus(Pose meanPose, Pose stdevPose, double rateContami){
        return impl->resetStatus(meanPose, stdevPose, rateContami);
    }

    bool StreamParticleFilter::resetStatus(const Beacons &beacons){
        return impl->resetStatus(beacons);
    }
    
    bool StreamParticleFilter::resetStatus(const Location &location, const Beacons &beacons){
        return impl->resetStatus(location, beacons);
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
    
    StreamParticleFilter& StreamParticleFilter::effectiveSampleSizeThreshold(double essThreshold){
        impl->essThreshold(essThreshold);
        return *this;
    }

    StreamParticleFilter& StreamParticleFilter::mixtureParameters(MixtureParameters mixParams){
        impl->mixtureParameters(mixParams);
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
    StreamParticleFilter& StreamParticleFilter::altitudeManager(std::shared_ptr<AltitudeManager> altitudeManager){
        impl->altitudeManager(altitudeManager);
        return *this;
    }

    StreamParticleFilter& StreamParticleFilter::systemModel(std::shared_ptr<SystemModel<State, SystemModelInput>> randomWalker){
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

    StreamParticleFilter& StreamParticleFilter::observationDependentInitializer(std::shared_ptr<ObservationDependentInitializer<State, Beacons>> metro){
        impl->observationDependentInitializer(metro);
        return * this;
    }
    
    StreamParticleFilter& StreamParticleFilter::posteriorResampler(PosteriorResampler<State>::Ptr posRes){
        impl->posteriorResampler(posRes);
        return * this;
    }

    StreamParticleFilter& StreamParticleFilter::dataStore(DataStore::Ptr dataStore){
        impl->dataStore(dataStore);
        return * this;
    }
    
    StreamParticleFilter& StreamParticleFilter::enablesFloorUpdate(bool enablesFloorUpdate){
        impl->enablesFloorUpdate(enablesFloorUpdate);
        return * this;
    }
    
    StreamParticleFilter& StreamParticleFilter::floorUpdateMode(FloorUpdateMode mode){
        impl->floorUpdateMode(mode);
        return * this;
    }    
    
    StreamParticleFilter& StreamParticleFilter::floorTransitionParameters(FloorTransitionParameters::Ptr params){
        impl->floorTransitionParameters(params);
        return * this;
    }
    
    StreamParticleFilter& StreamParticleFilter::locationStatusMonitorParameters(LocationStatusMonitorParameters::Ptr params){
        impl->locationStatusMonitorParameters(params);
        return * this;
    }
}
