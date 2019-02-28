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
#include "OrientationAdjuster.hpp"

#include "GaussianProcessLDPLMultiModel.hpp"

// for pose random walker in building
#include "Building.hpp"
#include "RandomWalker.hpp"
#include "RandomWalkerMotion.hpp"
#include "SystemModelInBuilding.hpp"
#include "AltitudeManagerSimple.hpp"

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
    class BasicLocalizerOptions;
    class BasicLocalizer;
    
    typedef struct {
        BasicLocalizer* localizer;
    } UserData;
    
    typedef enum {
        ONESHOT = 0,
        RANDOM_WALK = 1,
        RANDOM_WALK_ACC = 2,
        RANDOM_WALK_ACC_ATT = 3,
        WEAK_POSE_RANDOM_WALKER = 4
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
        
        // for observation model
        double coeffDiffFloorStdev = 5.0;
        int tDelay = -1; // 
        
        OrientationMeterType orientationMeterType = RAW_AVERAGE;

        // parameter objects
        PoseProperty::Ptr poseProperty = std::make_shared<PoseProperty>();
        StateProperty::Ptr stateProperty = std::make_shared<StateProperty>();
        
        StreamParticleFilter::FloorTransitionParameters::Ptr pfFloorTransParams = std::make_shared<StreamParticleFilter::FloorTransitionParameters>();
        LocationStatusMonitorParameters::Ptr locationStatusMonitorParameters = std::make_shared<LocationStatusMonitorParameters>();
        SystemModelInBuildingProperty::Ptr prwBuildingProperty = std::make_shared<SystemModelInBuildingProperty>();
        
        // yaw drift adjuster
        bool applysYawDriftAdjust = false;
        
        int adjustsBeaconSort = 0;
        
    protected:
        double meanRssiBias_ = 0.0;
        double minRssiBias_ = -10;
        double maxRssiBias_ = 10;
        
        double headingConfidenceForOrientationInit_ = 0.0;
        
    public:
                
        template<class Archive>
        void serialize(Archive & ar, std::uint32_t const version)
        {
            OPTIONAL_NVP(ar,nStates);
            OPTIONAL_NVP(ar,alphaWeaken);
            OPTIONAL_NVP(ar,nSmooth);
            OPTIONAL_NVP(ar,nSmoothTracking);
            
            OPTIONAL_NVP(ar,smoothType);
            OPTIONAL_NVP(ar,localizeMode);
            
            OPTIONAL_NVP(ar,effectiveSampleSizeThreshold);
            OPTIONAL_NVP(ar,nStrongest);
            OPTIONAL_NVP(ar,enablesFloorUpdate);
            
            
            OPTIONAL_NVP(ar,walkDetectSigmaThreshold);
            OPTIONAL_NVP(ar,meanVelocity);
            OPTIONAL_NVP(ar,stdVelocity);
            OPTIONAL_NVP(ar,diffusionVelocity);
            OPTIONAL_NVP(ar,minVelocity);
            OPTIONAL_NVP(ar,maxVelocity);
            
            
            OPTIONAL_NVP(ar,stdRssiBias);
            OPTIONAL_NVP(ar,diffusionRssiBias);
            OPTIONAL_NVP(ar,stdOrientation);
            OPTIONAL_NVP(ar,diffusionOrientationBias);
            
            OPTIONAL_NVP(ar,angularVelocityLimit);
            
            // Parametes for PoseRandomWalker
            OPTIONAL_NVP(ar,doesUpdateWhenStopping);
            OPTIONAL_NVP(ar,maxIncidenceAngle);
            OPTIONAL_NVP(ar,weightDecayHalfLife);
            
            // Parameters for RandomWalkerMotion and WeakPoseRandomWalker
            OPTIONAL_NVP(ar,sigmaStop);
            OPTIONAL_NVP(ar,sigmaMove);
            
            // Parameters for SystemModelInBuilding
            OPTIONAL_NVP(ar,velocityRateFloor);
            OPTIONAL_NVP(ar,velocityRateElevator);
            OPTIONAL_NVP(ar,velocityRateStair);
            OPTIONAL_NVP(ar,velocityRateEscalator);
            OPTIONAL_NVP(ar,relativeVelocityEscalator);
            
            // Parameters for WeakPoseRandomWalker
            OPTIONAL_NVP(ar,probabilityOrientationBiasJump);
            OPTIONAL_NVP(ar,poseRandomWalkRate);
            OPTIONAL_NVP(ar,randomWalkRate);
            OPTIONAL_NVP(ar,probabilityBackwardMove);
            
            OPTIONAL_NVP(ar,nBurnIn);
            OPTIONAL_NVP(ar,burnInRadius2D);
            OPTIONAL_NVP(ar,burnInInterval);
            OPTIONAL_NVP(ar,burnInInitType);
            
            OPTIONAL_NVP(ar,mixProba);
            OPTIONAL_NVP(ar,rejectDistance);
            OPTIONAL_NVP(ar,rejectFloorDifference);
            OPTIONAL_NVP(ar,nBeaconsMinimum);
            
            OPTIONAL_NVP(ar,locLB);
            
            OPTIONAL_NVP(ar,usesAltimeterForFloorTransCheck);
            OPTIONAL_NVP(ar,coeffDiffFloorStdev);
            
            OPTIONAL_NVP(ar,orientationMeterType);
            
            // parameter objects
            OPTIONAL_NVP(ar,*poseProperty);
            OPTIONAL_NVP(ar,*stateProperty);
            
            OPTIONAL_NVP(ar,*pfFloorTransParams);
            OPTIONAL_NVP(ar,*locationStatusMonitorParameters);
            OPTIONAL_NVP(ar,*prwBuildingProperty);
            
            // protected
            OPTIONAL_NVP(ar,meanRssiBias_);
            OPTIONAL_NVP(ar,minRssiBias_);
            OPTIONAL_NVP(ar,maxRssiBias_);
            OPTIONAL_NVP(ar,headingConfidenceForOrientationInit_);
            
            if(1<=version){
                OPTIONAL_NVP(ar,applysYawDriftAdjust);
            }
            
            OPTIONAL_NVP(ar, adjustsBeaconSort);
            
            try{
                ar(CEREAL_NVP(tDelay));
            }catch(cereal::Exception& e){
                std::cerr << "tDelay is not found in config and not updated." << std::endl;
            }
        }
        
    };
    
    class BasicLocalizerOptions{
    public:
        GPType gpType = GPNORMAL;
        KNLType knlType = KNLALL;
        double overlapScale = 0.001;
        MatType matType = DENSE;
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
        LatLngConverter::Ptr latLngConverter_;

        LocalHeadingBuffer mLocalHeadingBuffer = LocalHeadingBuffer(10); // buffer size 10 is not important.
        
        Status::LocationStatus mLocationStatus = Status::UNKNOWN;
        
        bool mDisableAcceleration = false;
        
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
        
        bool forceTraining = false;
        bool binaryOutput = false;
        std::string binaryFile = "";
        std::string trainedFile = "";
        std::string finalizedFile = "";
        BasicLocalizerOptions basicLocalizerOptions;
        
        std::set<std::string> binarizeTargets{"model"}; // "model", "building"
        
        bool finalizeMapdata = false;
        
        OrientationMeterAverageParameters orientationMeterAverageParameters;
        std::shared_ptr<OrientationMeter> orientationMeter;
        
        PedometerWalkingStateParameters pedometerWSParams;
        std::shared_ptr<Pedometer> pedometer;
        
        // parameter objects
        PoseRandomWalkerProperty::Ptr poseRandomWalkerProperty = std::make_shared<PoseRandomWalkerProperty>();
        WeakPoseRandomWalkerProperty::Ptr wPRWproperty = std::make_shared<WeakPoseRandomWalkerProperty>();
        AltitudeManagerSimple::Parameters::Ptr altimeterManagerParameters = std::make_shared<AltitudeManagerSimple::Parameters>();
        
        // system models
        std::shared_ptr<RandomWalker<State, SystemModelInput>>randomWalker;
        std::shared_ptr<PoseRandomWalker>poseRandomWalker;
        std::shared_ptr<PoseRandomWalkerInBuilding> poseRandomWalkerInBuilding;
        
        std::shared_ptr<Resampler<State>> resampler;
        
        std::shared_ptr<StatusInitializerImpl> statusInitializer;
        
        std::shared_ptr<BeaconFilter> beaconFilter;
        
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
        
        bool resetAllStatus();

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
        
        // for yaw drift adjuster
        OrientationDriftAdjusterSimple::Ptr yawDriftAdjuster = std::make_shared<OrientationDriftAdjusterSimple>();
        
        // control disable/enable acceleration
        void disableAcceleration(bool, long);
    };
}

// assign version
CEREAL_CLASS_VERSION(loc::BasicLocalizerParameters, 1);
#endif /* BasicLocalizerBuilder_hpp */
