/*******************************************************************************
 * Copyright (c) 2014, 2016  IBM Corporation and others
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

#ifndef BasicLocalizer_hpp
#define BasicLocalizer_hpp

#include <iostream>
#include <string>

#include <boost/circular_buffer.hpp>

#include "StreamLocalizer.hpp"
#include "StreamParticleFilter.hpp"

#include "PoseRandomWalker.hpp"
#include "WeakPoseRandomWalker.hpp"

#include "GridResampler.hpp"
#include "StatusInitializerStub.hpp"
#include "StatusInitializerImpl.hpp"

#include "DataStore.hpp"
#include "DataStoreImpl.hpp"

#include "OrientationMeterAverage.hpp"
#include "PedometerWalkingState.hpp"

#include "GaussianProcessLDPLMultiModel.hpp"

// for pose random walker in building
#include "Building.hpp"
#include "RandomWalker.hpp"
#include "RandomWalkerMotion.hpp"
#include "SystemModelInBuilding.hpp"

#include "BeaconFilterChain.hpp"
#include "CleansingBeaconFilter.hpp"
#include "StrongestBeaconFilter.hpp"

#include "ObservationDependentInitializer.hpp"
#include "MetropolisSampler.hpp"

#include "SerializeUtils.hpp"
#include "LatLngConverter.hpp"

#define N_SMOOTH_MAX 10

namespace loc {
    
    class BasicLocalizerParameters;
    class BasicLocalizer;
    
    typedef struct {
        BasicLocalizer* localizer;
    } UserData;
    
    typedef enum {
        ONESHOT,
        RANDOM_WALK,
        RANDOM_WALK_ACC,
        RANDOM_WALK_ACC_ATT,
        WEAK_POSE_RANDOM_WALKER
    } LocalizeMode;
    
    /*
    typedef enum {
        UNKNOWN,
        LOCATING,
        TRACKING
    } LocalizeState;
    */
    
    typedef enum {
        NORMAL, TDIST
    } NormalFunction;
    
    typedef enum {
        SMOOTH_LOCATION,
        SMOOTH_RSSI
    } SmoothType;
    
    typedef enum{
        RAW_AVERAGE,
        TRANSFORMED_AVERAGE
    } OrientationMeterType;
    
    class LocalHeadingBuffer{
    protected:
        boost::circular_buffer<LocalHeading> buffer_;
        std::mutex mtx_;
    public:
        LocalHeadingBuffer(size_t);
        ~LocalHeadingBuffer() = default;
        LocalHeadingBuffer(const LocalHeadingBuffer&);
        LocalHeadingBuffer& operator=(const LocalHeadingBuffer&);
        void push_back(const LocalHeading&);
        LocalHeading& back();
        size_t size();
    };
    
    
    struct UserDataBridge{
        UserData* userData;
        void (*functionCalledAfterUpdateWithPtr)(void*, Status*) = NULL;
        BasicLocalizer* basicLocalizer;
    };
    
    class BasicLocalizerParameters{
    public:
        int nStates = 1000;
        double alphaWeaken = 0.3;
        int nSmooth = 10;
        int nSmoothTracking = 1;
        SmoothType smoothType = SMOOTH_LOCATION;
        LocalizeMode localizeMode = ONESHOT;
        
        double effectiveSampleSizeThreshold = 1000;
        int nStrongest = 10;
        bool enablesFloorUpdate = true;
        
        double walkDetectSigmaThreshold = 0.6;
        double meanVelocity = 1.0;
        double stdVelocity = 0.3;
        double diffusionVelocity = 0.1;
        double minVelocity = 0.1;
        double maxVelocity = 1.5;
        
        double stdRssiBias = 2.0;
        double diffusionRssiBias = 0.2;
        double stdOrientation = 3.0;
        double diffusionOrientationBias = 10;
        
        double angularVelocityLimit = 30;
        // Parametes for PoseRandomWalker
        bool doesUpdateWhenStopping = false;
        
        double maxIncidenceAngle = 45;
        double weightDecayHalfLife = 5; // 0.87055056329; // this^5 = 0.5
        
        // Parameters for RandomWalkerMotion and WeakPoseRandomWalker
        double sigmaStop = 0.1;
        double sigmaMove = 1.0;
        
        // Parameters for SystemModelInBuilding
        double velocityRateFloor = 1.0;
        double velocityRateElevator = 0.5;
        double velocityRateStair = 0.5;
        double velocityRateEscalator = 0.5;
        double relativeVelocityEscalator = 0.4; // m/s
        
        // Parameters for WeakPoseRandomWalker
        double probabilityOrientationBiasJump = 0.1;
        double poseRandomWalkRate = 1.0;
        double randomWalkRate = 0.2;
        double probabilityBackwardMove = 0.0;
        
        int nBurnIn = 1000;
        int burnInRadius2D = 10;
        int burnInInterval = 1;
        InitType burnInInitType = INIT_WITH_SAMPLE_LOCATIONS;
        
        double mixProba = 0.000;
        double rejectDistance = 5;
        double rejectFloorDifference = 0.99;
        int nBeaconsMinimum = 3;
        
        Location locLB{0.5, 0.5, 1e-6, 1e-6};
        
        bool usesAltimeterForFloorTransCheck = false;
        double coeffDiffFloorStdev = 5.0;
        
        OrientationMeterType orientationMeterType = RAW_AVERAGE;

        // parameter objects
        PoseProperty::Ptr poseProperty = std::make_shared<PoseProperty>();
        StateProperty::Ptr stateProperty = std::make_shared<StateProperty>();
        
        StreamParticleFilter::FloorTransitionParameters::Ptr pfFloorTransParams = std::make_shared<StreamParticleFilter::FloorTransitionParameters>();
        LocationStatusMonitorParameters::Ptr locationStatusMonitorParameters = std::make_shared<LocationStatusMonitorParameters>();
        SystemModelInBuildingProperty::Ptr prwBuildingProperty = std::make_shared<SystemModelInBuildingProperty>();

    protected:
        double meanRssiBias_ = 0.0;
        double minRssiBias_ = -10;
        double maxRssiBias_ = 10;
        
        double headingConfidenceForOrientationInit_ = 0.0;
        
    public:
        template<class Archive>
        void serialize(Archive & ar, std::uint32_t const version)
        {
            if (0 <= version) {
                ar(CEREAL_NVP(nStates));
                ar(CEREAL_NVP(alphaWeaken));
                ar(CEREAL_NVP(nSmooth));
                ar(CEREAL_NVP(nSmoothTracking));
                
                ar(CEREAL_NVP(smoothType));
                ar(CEREAL_NVP(localizeMode));
                
                ar(CEREAL_NVP(effectiveSampleSizeThreshold));
                ar(CEREAL_NVP(nStrongest));
                ar(CEREAL_NVP(enablesFloorUpdate));
                
                
                ar(CEREAL_NVP(walkDetectSigmaThreshold));
                ar(CEREAL_NVP(meanVelocity));
                ar(CEREAL_NVP(stdVelocity));
                ar(CEREAL_NVP(diffusionVelocity));
                ar(CEREAL_NVP(minVelocity));
                ar(CEREAL_NVP(maxVelocity));
                
                
                ar(CEREAL_NVP(stdRssiBias));
                ar(CEREAL_NVP(diffusionRssiBias));
                ar(CEREAL_NVP(stdOrientation));
                ar(CEREAL_NVP(diffusionOrientationBias));
                
                ar(CEREAL_NVP(angularVelocityLimit));
                
                // Parametes for PoseRandomWalker
                ar(CEREAL_NVP(doesUpdateWhenStopping));
                ar(CEREAL_NVP(maxIncidenceAngle));
                ar(CEREAL_NVP(weightDecayHalfLife));
                
                // Parameters for RandomWalkerMotion and WeakPoseRandomWalker
                ar(CEREAL_NVP(sigmaStop));
                ar(CEREAL_NVP(sigmaMove));
                
                // Parameters for SystemModelInBuilding
                ar(CEREAL_NVP(velocityRateFloor));
                ar(CEREAL_NVP(velocityRateElevator));
                ar(CEREAL_NVP(velocityRateStair));
                ar(CEREAL_NVP(velocityRateEscalator));
                ar(CEREAL_NVP(relativeVelocityEscalator));
                
                // Parameters for WeakPoseRandomWalker
                ar(CEREAL_NVP(probabilityOrientationBiasJump));
                ar(CEREAL_NVP(poseRandomWalkRate));
                ar(CEREAL_NVP(randomWalkRate));
                ar(CEREAL_NVP(probabilityBackwardMove));

                ar(CEREAL_NVP(nBurnIn));
                ar(CEREAL_NVP(burnInRadius2D));
                ar(CEREAL_NVP(burnInInterval));
                ar(CEREAL_NVP(burnInInitType));
                
                ar(CEREAL_NVP(mixProba));
                ar(CEREAL_NVP(rejectDistance));
                ar(CEREAL_NVP(rejectFloorDifference));
                ar(CEREAL_NVP(nBeaconsMinimum));

                ar(CEREAL_NVP(locLB));

                ar(CEREAL_NVP(usesAltimeterForFloorTransCheck));
                ar(CEREAL_NVP(coeffDiffFloorStdev));

                ar(CEREAL_NVP(orientationMeterType));

                // parameter objects
                ar(CEREAL_NVP(*poseProperty));
                ar(CEREAL_NVP(*stateProperty));
                
                ar(CEREAL_NVP(*pfFloorTransParams));
                ar(CEREAL_NVP(*locationStatusMonitorParameters));
                ar(CEREAL_NVP(*prwBuildingProperty));
            }
        }
        
    };
    
    class BasicLocalizer: public StreamLocalizer, public BasicLocalizerParameters{
        
    private:
        std::shared_ptr<StreamParticleFilter> mLocalizer;
        std::shared_ptr<GaussianProcessLDPLMultiModel<State, Beacons>> deserializedModel;
        UserData userData;
        double isReady = false;
        
        void (*mFunctionCalledAfterUpdate)(Status*) = NULL;
        void (*mFunctionCalledAfterUpdate2)(void*, Status*) = NULL;
        void (*mFunctionCalledToLog)(void*, std::string) = NULL;
        void *mUserData = NULL;
        void *mUserDataToLog = NULL;
        
        UserDataBridge userDataBridge;
        void *mUserDataBridge = NULL;
        
        //std::shared_ptr<loc::Status> mResult;
        std::shared_ptr<Status> mTrackedStatus;
        
        std::vector<loc::State> status_list[N_SMOOTH_MAX];
        std::vector<loc::Beacon> beacons_list[N_SMOOTH_MAX];
        
        int smooth_count = 0;
        double mEstimatedRssiBias = 0;
        
        bool isTrackingLocalizer() {
            switch(localizeMode) {
                case ONESHOT:
                    return false;
                case RANDOM_WALK:
                case RANDOM_WALK_ACC:
                case RANDOM_WALK_ACC_ATT:
                case WEAK_POSE_RANDOM_WALKER:
                default:
                    return true;
            }
        };
        
        void updateStateProperty();
        
        Anchor anchor;
        LatLngConverter::Ptr latLngConverter_ = LatLngConverter::Ptr(new LatLngConverter());
        
        LocalHeadingBuffer mLocalHeadingBuffer = LocalHeadingBuffer(10); // buffer size 10 is not important.
        
        Status::LocationStatus mLocationStatus = Status::UNKNOWN;
        
    public:
        BasicLocalizer();
        ~BasicLocalizer();
        
        BasicLocalizer(const BasicLocalizerParameters& params): BasicLocalizerParameters(params){
            this->meanRssiBias(meanRssiBias_);
            this->minRssiBias(minRssiBias_);
            this->maxRssiBias(maxRssiBias_);
        }
        
        bool isVerboseLocalizer = false;
        
        std::shared_ptr<DataStoreImpl> dataStore;
        
        void normalFunction(NormalFunction type, double option);
        void meanRssiBias(double b);
        void maxRssiBias(double b);
        void minRssiBias(double b);
        /*
         void angularVelocityLimit(double a);
         */
        double estimatedRssiBias();
        
        OrientationMeterAverageParameters orientationMeterAverageParameters;
        std::shared_ptr<OrientationMeter> orientationMeter;
        
        PedometerWalkingStateParameters pedometerWSParams;
        std::shared_ptr<Pedometer> pedometer;
        
        // parameter objects
        PoseRandomWalkerProperty::Ptr poseRandomWalkerProperty = std::make_shared<PoseRandomWalkerProperty>();
        WeakPoseRandomWalkerProperty::Ptr wPRWproperty = std::make_shared<WeakPoseRandomWalkerProperty>();
        
        // system models
        std::shared_ptr<RandomWalker<State, SystemModelInput>>randomWalker;
        std::shared_ptr<PoseRandomWalker>poseRandomWalker;
        std::shared_ptr<PoseRandomWalkerInBuilding> poseRandomWalkerInBuilding;
        
        std::shared_ptr<Resampler<State>> resampler;
        
        std::shared_ptr<StatusInitializerImpl> statusInitializer;
        
        std::shared_ptr<StrongestBeaconFilter> beaconFilter;
        
        loc::Pose stdevPose;
        
        std::shared_ptr<MetropolisSampler<State, Beacons>> obsDepInitializer;
        
        MetropolisSampler<State, Beacons>::Parameters msParams;
        
        LatLngConverter::Ptr latLngConverter();

        StreamLocalizer& updateHandler(void (*functionCalledAfterUpdate)(Status*)) override;
        StreamLocalizer& updateHandler(void (*functionCalledAfterUpdate)(void*, Status*), void* inUserData) override;
        
        StreamLocalizer& logHandler(void (*functionCalledToLog)(void*, std::string), void* inUserData);

        StreamLocalizer& putAttitude(const Attitude attitude) override;
        StreamLocalizer& putAcceleration(const Acceleration acceleration) override;
        StreamLocalizer& putBeacons(const Beacons beacons) override;
        // StreamLocalizer& putWiFiAPs(const WiFiAPs wifiaps) override;
        StreamLocalizer& putLocalHeading(const LocalHeading heading) override;
        StreamLocalizer& putHeading(const Heading heading);
        StreamLocalizer& putAltimeter(const Altimeter altimeter) override;
        Status* getStatus() override;
                                 
        bool resetStatus() override;
        bool resetStatus(Pose pose) override;
        bool resetStatus(Pose meanPose, Pose stdevPose) override;
        bool resetStatus(Pose meanPose, Pose stdevPose, double rateContami) override;
        bool resetStatus(const Beacons& beacons) override;
        bool resetStatus(const Location& location, const Beacons& beacons) override;

        BasicLocalizer& setModel(std::string modelPath, std::string workingDir);
        
        bool tracksOrientation(){
            switch(localizeMode) {
                case ONESHOT:
                case RANDOM_WALK:
                case RANDOM_WALK_ACC:
                    return false;
                case RANDOM_WALK_ACC_ATT:
                case WEAK_POSE_RANDOM_WALKER:
                    return true;
                default:
                    return false;
            }
        };
        
        std::shared_ptr<GaussianProcessLDPLMultiModel<State, Beacons>> observationModel() const{
            return deserializedModel;
        };
        
        // for orientation initialization
        void headingConfidenceForOrientationInit(double confidence);
        
        void updateLocationStatus(Status*);
        void overwriteLocationStatus(Status::LocationStatus);
        
    };
}

// assign version
CEREAL_CLASS_VERSION(loc::BasicLocalizerParameters, 0);
#endif /* BasicLocalizerBuilder_hpp */
