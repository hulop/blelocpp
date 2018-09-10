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

#include "BasicLocalizer.hpp"
#include <boost/archive/iterators/binary_from_base64.hpp>
#include <boost/archive/iterators/transform_width.hpp>

#include <iostream>
#include <chrono>
#include "LogUtil.hpp"
#include "WeakPoseRandomWalker.hpp"
#include "AltitudeManagerSimple.hpp"

#include "TransformedOrientationMeterAverage.hpp"
#include "BeaconFilterChain.hpp"
#include "RegisteredBeaconFilter.hpp"

namespace loc{
    // BasicLocalizer
    BasicLocalizer::BasicLocalizer(){
        //pfFloorTransParams = StreamParticleFilter::FloorTransitionParameters::Ptr(new StreamParticleFilter::FloorTransitionParameters);
    }
    BasicLocalizer::~BasicLocalizer(){
    }
    
    StreamLocalizer& BasicLocalizer::updateHandler(void (*functionCalledAfterUpdate)(Status*)) {
        mFunctionCalledAfterUpdate = functionCalledAfterUpdate;
        if (mLocalizer) {
            mLocalizer->updateHandler(mFunctionCalledAfterUpdate);
        }
        return *this;
    }
    
    void bridgeFunctionCalledAfterUpdate2(void* userDataBridge, Status* status){
        UserDataBridge* udb = (UserDataBridge*) userDataBridge;
        
        BasicLocalizer* localizer = udb->basicLocalizer;
        UserData* userData = udb->userData;
        
        localizer->updateLocationStatus(localizer->getStatus());
        
        udb->functionCalledAfterUpdateWithPtr(userData, status);
    }
    
    StreamLocalizer& BasicLocalizer::updateHandler(void (*functionCalledAfterUpdate)(void*, Status*), void* inUserData) {
        mFunctionCalledAfterUpdate2 = functionCalledAfterUpdate;
        mUserData = inUserData;
        
        userDataBridge.userData = (UserData*) inUserData;
        userDataBridge.functionCalledAfterUpdateWithPtr = mFunctionCalledAfterUpdate2;
        userDataBridge.basicLocalizer = this;
        
        mUserDataBridge = &userDataBridge;
        
        if (mLocalizer) {
            mLocalizer->updateHandler(bridgeFunctionCalledAfterUpdate2, mUserDataBridge);
        }
        return *this;
    }
    
    StreamLocalizer& BasicLocalizer::logHandler(void (*functionCalledToLog)(void*, std::string), void* inUserData) {
        mFunctionCalledToLog = functionCalledToLog;
        mUserDataToLog = inUserData;
        return *this;
    }
    
    StreamLocalizer& BasicLocalizer::putAttitude(const Attitude attitude) {
        if (!isReady) {
            return *this;
        }
        if (mFunctionCalledToLog) {
            mFunctionCalledToLog(mUserDataToLog, LogUtil::toString(attitude));
        }
        Attitude altTmp(attitude);
        if(orientationMeterType==TRANSFORMED_AVERAGE){
            double yawTransformed = TransformedOrientationMeterAverage::transformOrientation(attitude);
            altTmp.yaw(yawTransformed);
        }
        if(yawDriftAdjuster){
            auto altAdj = yawDriftAdjuster->adjustAttitude(altTmp);
            double drift = yawDriftAdjuster->getCurrentDrift();
            double driftRate = yawDriftAdjuster->getCurrentDriftRate();
            if(isVerboseLocalizer){
                std::cout << "EstimatedYawDrift(timestamp,drift,driftRate,isApplied): "<< attitude.timestamp() << "," << drift << "," << driftRate << "," << applysYawDriftAdjust << std::endl;
            }
            if(applysYawDriftAdjust){
                altTmp = altAdj;
            }
        }
        if (!isTrackingLocalizer()) {
            return *this;
        }
        mLocalizer->putAttitude(altTmp);
        return *this;
    }
    StreamLocalizer& BasicLocalizer::putAcceleration(const Acceleration acceleration) {
        if (!isReady) {
            return *this;
        }
        if (mFunctionCalledToLog) {
            mFunctionCalledToLog(mUserDataToLog, LogUtil::toString(acceleration));
        }
        if (!isTrackingLocalizer()) {
            return *this;
        }
        else{
            Acceleration accTmp = acceleration;
            if(mDisableAcceleration){
                accTmp.ax(0.0)->ay(0.0)->az(0.0);
            }
            if(mLocationStatus==Status::STABLE or mLocationStatus==Status::UNSTABLE){
                mLocalizer->putAcceleration(accTmp);
            }
        }
        return *this;
    }
    
    StreamLocalizer& BasicLocalizer::putLocalHeading(const LocalHeading localHeading) {
        // pass
        return *this;
    }
    
    
    StreamLocalizer& BasicLocalizer::putHeading(const Heading heading) {
        if (mFunctionCalledToLog) {
            mFunctionCalledToLog(mUserDataToLog, LogUtil::toString(heading));
        }
        auto cvt = latLngConverter();
        if(cvt){
            auto localHeading = cvt->headingGlobalToLocal(heading);
            mLocalHeadingBuffer.push_back(localHeading);
        }
        if(yawDriftAdjuster){
            yawDriftAdjuster->putHeading(heading);
        }
        return *this;
    }
    
    StreamLocalizer& BasicLocalizer::putAltimeter(const Altimeter altimeter){
        if (mFunctionCalledToLog) {
            mFunctionCalledToLog(mUserDataToLog, LogUtil::toString(altimeter));
        }
        if (!isReady) {
            return *this;
        }
        if (!isTrackingLocalizer()) {
            return *this;
        }
        else{
            if(mLocationStatus==Status::STABLE or mLocationStatus==Status::UNSTABLE){
                mLocalizer->putAltimeter(altimeter);
            }
        }
        return *this;
    }
    
    Beacons smoothBeaconsList(const std::vector<Beacon>* beacons_list , int smooth_count, int nSmooth){
        std::map<BeaconId, loc::Beacons> allBeacons;
        
        for(int i = 0; i < N_SMOOTH_MAX && i < smooth_count+1 && i < nSmooth; i++) {
            for(auto& b: beacons_list[i]) {
                if (b.rssi() == 0) {
                    continue;
                }
                auto iter = allBeacons.find(b.id());
                if (iter == allBeacons.end()) {
                    auto bs = loc::Beacons();
                    bs.insert(bs.end(), b);
                    allBeacons.insert(std::make_pair(b.id(), bs));
                }else{
                    iter->second.insert(iter->second.end(), b);
                }
            }
        }
        
        Beacons beaconsAveraged;
        for(auto itr = allBeacons.begin(); itr != allBeacons.end(); ++itr) {
            const auto& id = itr->first;
            const auto& uuid = id.uuid();
            auto major = id.major();
            auto minor = id.minor();
            auto& bs = itr->second;
            int c = 0;
            double rssi = 0;
            for(auto& b:bs) {
                rssi += b.rssi();
                c++;
            }
            beaconsAveraged.insert(beaconsAveraged.end(), loc::Beacon(uuid, major, minor, rssi/c));
        }
        return beaconsAveraged;
    }
    
    /*
    bool checkStatesInStdev2D(const std::vector<State>& states, double stdevLimit){
        double var2D = Location::compute2DVariance(states);
        double stdev2D = std::pow(var2D, 1.0/4.0);
        std::cout << "stdev2D = " << stdev2D << std::endl;
        if(stdev2D < stdevLimit){
            return true;
        }else{
            return false;
        }
    }
    */
    
    StreamLocalizer& BasicLocalizer::putBeacons(const Beacons beacons) {
        if (!isReady) {
            return *this;
        }
        if (mFunctionCalledToLog) {
            mFunctionCalledToLog(mUserDataToLog, LogUtil::toString(beacons));
        }
        if (beaconFilter->filter(beacons).size() ==0){
            std::cout << "The number of strong beacon is zero." << std::endl;
            return *this;
        }
        Beacons beaconsTmp = beacons;
        if (smoothType == SMOOTH_RSSI) {
            beacons_list[(smooth_count)%std::min(N_SMOOTH_MAX,nSmooth)] = beacons;
            Beacons newBeacons = smoothBeaconsList(beacons_list, smooth_count, nSmooth);
            newBeacons.timestamp(beacons.timestamp());
            beaconsTmp = newBeacons;
            smooth_count++;
        }
        /*
            switch(mState) {
                case UNKNOWN:
                    mLocalizer->resetStatus(beacons);
                    break;
                case LOCATING:
                    mLocalizer->resetStatus(beacons);
                    break;
                case TRACKING:
                    mLocalizer->putBeacons(beacons);
                    break;
            }
        */
        
        if(isVerboseLocalizer){
            if(mTrackedStatus){
                std::cout << "mTrackedStatus exists" << std::endl;
            }else{
                std::cout << "mTrackedStatus does not exist" << std::endl;
            }
        }
        
        mLocalizer->getStatus()->locationStatus(mLocationStatus); // ensure innerStatus == mLocationStatus before put method
        
        if(isTrackingLocalizer()){
            switch(mLocationStatus){
                case(Status::UNKNOWN): case(Status::LOCATING):
                    mLocalizer->resetStatus(beaconsTmp);
                    break;
                case(Status::STABLE): case(Status::UNSTABLE):
                    mLocalizer->putBeacons(beaconsTmp);
                    if(!mTrackedStatus){
                        mTrackedStatus = std::make_shared<Status>();
                    }
                    *mTrackedStatus = *mLocalizer->getStatus();
                    break;
                case(Status::NIL):
                    BOOST_THROW_EXCEPTION(LocException("location status is not set (NIL)"));
                    break;
            }
        }else{
            switch(mLocationStatus){
                case(Status::UNKNOWN): case(Status::LOCATING): case(Status::STABLE): case(Status::UNSTABLE):
                    mLocalizer->resetStatus(beaconsTmp);
                    break;
                case(Status::NIL):
                    BOOST_THROW_EXCEPTION(LocException("location status is not set (NIL)"));
                    break;
            }
        }
        
        if(smoothType == SMOOTH_LOCATION
           && (mLocationStatus==Status::STABLE || mLocationStatus==Status::UNSTABLE )
           && isTrackingLocalizer()){
            return *this; // SMOOTH_LOCATION & TRACKING ends here.
        }
        
        bool isStatesConverged = false;
        
        std::function<NormalParameter()> computeNormalParameterForInit = [&]{
            double ori;
            double oridev;
            if(mTrackedStatus){
                auto yaw = orientationMeter->getYaw(); // orientationMeter is always updated in putAttitude.
                auto trackedStates = mTrackedStatus->states();
                std::vector<double> orientations;
                for(const auto& s: *trackedStates){
                    orientations.push_back(s.orientationBias());
                }
                auto wnp = MathUtils::computeWrappedNormalParameters(orientations);
                ori = yaw - wnp.mean();
                oridev = wnp.stdev();
            }else{
                if(mLocalHeadingBuffer.size()>0){
                    LocalHeading locHead = mLocalHeadingBuffer.back();
                    ori = locHead.orientation();
                    double largeOridev = 10.0 * M_PI;
                    oridev = locHead.orientationDeviation()>0 ? locHead.orientationDeviation() : largeOridev;
                }else{
                    ori = std::numeric_limits<double>::quiet_NaN();
                    oridev = std::numeric_limits<double>::infinity();
                    BOOST_THROW_EXCEPTION(LocException("Heading has not been input."));
                }
            }
            
            NormalParameter wnp(ori, oridev);
            if(isVerboseLocalizer){
                if(mTrackedStatus){
                    std::cout << "orientation updated by tracked states (mu,sigma)=" << wnp.mean() << "," << wnp.stdev() << ")" << std::endl;
                }else{
                    std::cout << "orientation updated by device heading (mu,sigma)=" << wnp.mean() << "," << wnp.stdev() << ")" << std::endl;
                }
            }
            return wnp;
        };
        
        
        auto statusLatest = mLocalizer->getStatus();
        auto mResult = std::make_shared<Status>(); // local
        if (smoothType == SMOOTH_LOCATION) {
            *mResult = *statusLatest;
            ////loc::Status *status = new loc::Status(*statusLatest);
            
            int nSmoothTmp;
            //if (isTrackingLocalizer() && mLocationStatus==Status::STABLE) {
            if (mLocationStatus==Status::STABLE || mLocationStatus==Status::UNSTABLE) {
                nSmoothTmp = nSmoothTracking;
            }else{
                nSmoothTmp = nSmooth;
            }
            
            status_list[(smooth_count++)%std::min(N_SMOOTH_MAX,nSmoothTmp)] = *statusLatest->states();
            
            std::shared_ptr<States> states (new std::vector<loc::State>);
            double meanBias = 0;
            for(int i = 0; i < N_SMOOTH_MAX && i < smooth_count && i < nSmoothTmp; i++) {
                for(auto& s: status_list[i]) {
                    states->push_back(s);
                    meanBias += s.rssiBias();
                }
            }
            mEstimatedRssiBias = meanBias / states->size();
            
            // update orientation by heading or tracked orientation
            bool headingConfidenceIsActive = 0.0<headingConfidenceForOrientationInit_ && headingConfidenceForOrientationInit_<=1.0;
            if(headingConfidenceIsActive){
                auto wnp = computeNormalParameterForInit();
                RandomGenerator randGen;
                double contamiRate =  std::max(1.0-headingConfidenceForOrientationInit_, 0.0);
                for(auto& s: *states){
                    if(randGen.nextDouble() < contamiRate || std::isinf(wnp.stdev())){
                        s.orientation(Pose::normalizeOrientaion(2.0*M_PI*randGen.nextDouble()));
                    }else{
                        s.orientation(randGen.nextWrappedNormal(wnp.mean(), wnp.stdev()));
                    }
                }
            }
            
            mResult->states(states, Status::RESET);
            mResult->locationStatus(mLocationStatus);
            
            updateLocationStatus(mResult.get());
            if(mResult->locationStatus() == Status::STABLE){
                isStatesConverged = true;
            }
        } else {
            mResult.reset(statusLatest);
            BOOST_THROW_EXCEPTION(LocException("smoothType!=SMOOTH_LOCATION is not supported"));
        }

        if (mFunctionCalledAfterUpdate) {
            mFunctionCalledAfterUpdate(mResult.get());
        }
        
        if (mFunctionCalledAfterUpdate2 && mUserData) {
            mFunctionCalledAfterUpdate2(mUserData, mResult.get());
        }
        
        //if (isTrackingLocalizer() && smooth_count >= nSmooth && mState != TRACKING) {
        if(!isTrackingLocalizer()){
            return *this;
        }
        //if (isTrackingLocalizer() && isStatesConverged && mLocationStatus!=Status::STABLE) {
        if (isTrackingLocalizer() && isStatesConverged) {
            Pose refPose = *mResult->meanPose();
            std::vector<State> states = *mResult->states();
            auto idx = Location::findClosestLocationIndex(refPose, states);
            Location locClosest = states.at(idx);
            refPose.copyLocation(locClosest);

            auto std = loc::Location::standardDeviation(*mResult->states());
            refPose.floor(roundf(refPose.floor()));
            loc::Pose stdevPose;
            double largeOridev = 10*M_PI;
            stdevPose.x(std.x()).y(std.y()).orientation(largeOridev);
            
            mLocationStatus = Status::STABLE;
            
            // update orientation by heading or tracked orientation
            bool headingConfidenceIsActive = 0.0<headingConfidenceForOrientationInit_ && headingConfidenceForOrientationInit_<=1.0;
            if(headingConfidenceIsActive){
                auto wnp = computeNormalParameterForInit();
                refPose.orientation(wnp.mean());
                stdevPose.orientation(wnp.stdev());
                double contamiRate =  std::max(1.0-headingConfidenceForOrientationInit_, 0.0);
                mLocalizer->getStatus()->locationStatus(mLocationStatus);// must overwrite locationStatus before resetStatus because the callback function is called in resetStatus.
                mLocalizer->resetStatus(refPose, stdevPose, contamiRate);
                std::cout << "Reset=" << refPose << ", STD=" << std
                            << " with orientation(" << refPose.orientation() << "," << stdevPose.orientation() << ")" << std::endl;
            }else{
                mLocalizer->getStatus()->locationStatus(mLocationStatus);
                mLocalizer->resetStatus(refPose, stdevPose);
                std::cout << "Reset=" << refPose << ", STD=" << std << std::endl;
            }
        }
        
        return *this;
    }
    
    Status::LocationStatus transitLocationStatus(const Status::LocationStatus& tempLocStatus, const States& states, const LocationStatusMonitorParameters& params){

        double std2DExitStable = params.stdev2DExitStable();
        double std2DEnterStable = params.stdev2DEnterStable();
        double std2DEnterLocating = params.stdev2DEnterLocating();
        double std2DExitLocating = params.stdev2DExitLocating();
        
        double std2D = std::pow(Location::compute2DVariance(states), 1.0/4.0);
        
        Status::LocationStatus newLocStatus = tempLocStatus;
        switch(tempLocStatus){
            case(Status::UNKNOWN):
                if(std2D < std2DEnterLocating){
                    newLocStatus = Status::LOCATING;
                }
                break;
            case(Status::LOCATING):
                if(std2DExitLocating < std2D){
                    newLocStatus = Status::UNKNOWN;
                }
                else if(std2D < std2DEnterStable){
                    newLocStatus = Status::STABLE;
                }
                break;
            case(Status::STABLE):
                if(std2DExitStable < std2D){
                    newLocStatus = Status::UNKNOWN;
                }
                break;
            case(Status::UNSTABLE):
                // release control to mLocalizer
                break;
            case(Status::NIL):
                BOOST_THROW_EXCEPTION(LocException("location status is not set (NIL)"));
                break;
        }
        return newLocStatus;
    }
    
    void BasicLocalizer::updateLocationStatus(Status* status){
        
        auto oldLocStatus = mLocationStatus;
        auto midLocStatus = status->locationStatus();
        Status::LocationStatus newLocStatus;
        
        bool innerStatusWasUpdated = midLocStatus!=oldLocStatus;
        if(innerStatusWasUpdated){
            newLocStatus = midLocStatus;
        } else {
            if(this->isVerboseLocalizer){
                double std2D = std::pow(Location::compute2DVariance(*status->states()), 1.0/4.0);
                auto stdLoc = Location::standardDeviation(*status->states());
                std::cout << "std2D=" << std2D << ",stdX="<< stdLoc.x() << ",stdY=" << stdLoc.y()  << std::endl;
            }
            auto tmpLocStatus = transitLocationStatus(midLocStatus, *status->states(), *locationStatusMonitorParameters);
            if(midLocStatus==Status::LOCATING && tmpLocStatus==Status::STABLE){
                if(smooth_count>=nSmooth){
                    newLocStatus = Status::STABLE;
                }else{
                    newLocStatus = Status::LOCATING;
                }
            }else{
                newLocStatus = tmpLocStatus;
            }
        }
    
        mLocationStatus = newLocStatus;
        status->locationStatus(mLocationStatus);
        
        // check if location status changed
        if(oldLocStatus!=mLocationStatus){
            std::cout << "locationStatus changed from " << Status::locationStatusToString(oldLocStatus) << " to " << Status::locationStatusToString(newLocStatus) << std::endl;
            if(mLocationStatus==Status::UNKNOWN){ // when locationStatus becomes UNKNOWN
                //TODO (this block can be moved to a public method)
                smooth_count = 0;
                if (mFunctionCalledAfterUpdate2 && mUserData) {
                    mFunctionCalledAfterUpdate2(mUserData, status);
                }
            }
        }
    }
    
    void BasicLocalizer::overwriteLocationStatus(Status::LocationStatus locStatus){
        this->getStatus()->locationStatus(locStatus);
        updateLocationStatus(this->getStatus());
    }
    
    Status* BasicLocalizer::getStatus() {
        return mLocalizer->getStatus();
    }
    
    bool BasicLocalizer::resetStatus() {
        return mLocalizer->resetStatus();
    }
    bool BasicLocalizer::resetStatus(Pose pose) {
        return mLocalizer->resetStatus(pose);
    }
    bool BasicLocalizer::resetStatus(Pose meanPose, Pose stdevPose) {
        return mLocalizer->resetStatus(meanPose, stdevPose);
    }
    bool BasicLocalizer::resetStatus(Pose meanPose, Pose stdevPose, double rateContami) {
        return mLocalizer->resetStatus(meanPose, stdevPose, rateContami);
    }
    bool BasicLocalizer::resetStatus(const Beacons& beacons) {
        return mLocalizer->resetStatus(beacons);
    }
    
    bool BasicLocalizer::resetStatus(const Location& location, const Beacons& beacons) {
        bool ret = mLocalizer->resetStatus(location, beacons);
        double meanBias = 0;
        for(loc::State s: *mLocalizer->getStatus()->states()) {
            meanBias += s.rssiBias();
        }
        mEstimatedRssiBias = meanBias / mLocalizer->getStatus()->states()->size();

        return ret;
    }
    
    const bool has(const picojson::value::object &obj, std::string key) {
        auto itr = obj.find(key);
        return itr != obj.end();
    }
    
    const picojson::value &get(const picojson::value::object &obj, std::string key){
        auto itr = obj.find(key);
        if (itr != obj.end()) {
            return itr->second;
        }
        throw "not found";
    }
    
    const std::string &getString(const picojson::value::object &obj, std::string key) {
        auto& value = get(obj, key);
        if (value.is<std::string>()) {
            return value.get<std::string>();
        }
        throw "non string value";
    }
    double getDouble(const picojson::value::object &obj, std::string key) {
        auto& value = get(obj, key);
        if (value.is<double>()) {
            return value.get<double>();
        }
        throw "non double value";
    }
    const picojson::value::object &getObject(const picojson::value::object &obj, std::string key) {
        auto& value = get(obj, key);
        if (value.is<picojson::value::object>()) {
            return value.get<picojson::value::object>();
        }
        throw "non object value";
    }
    const picojson::value::array &getArray(const picojson::value::object &obj, std::string key) {
        auto& value = get(obj, key);
        if (value.is<picojson::value::array>()) {
            return value.get<picojson::value::array>();
        }
        throw "non array value";
    }

    BasicLocalizer& BasicLocalizer::setModel(std::string modelPath, std::string workingDir) {
        auto s = std::chrono::system_clock::now();
        std::cerr << "start setModel" << std::endl;
        if (isReady) { // TODO support multiple models
            std::cerr << "Already model was set" << std::endl;
            return *this;
        }
        
        std::ifstream file;
        file.open(modelPath, std::ios::in);
        if(!file.is_open()){
            throw "model file not found at "+modelPath;
        }
        std::istreambuf_iterator<char> input(file);
        
        picojson::value v;
        std::string err;
        picojson::parse(v, input, std::istreambuf_iterator<char>(), &err);
        if (!err.empty()) {
            throw err+" with reading "+modelPath;
        }
        if (!v.is<picojson::object>()) {
            throw "invalid JSON";
        }
        file.close();
        auto msec = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()-s).count();
        std::cerr << "parse JSON: " << msec << "ms" << std::endl;
        
        picojson::value::object& json = v.get<picojson::object>();
        
        auto& anchor = getObject(json, "anchor");
        this->anchor.latlng.lat = getDouble(anchor, "latitude");
        this->anchor.latlng.lng = getDouble(anchor, "longitude");
        this->anchor.rotate = getDouble(anchor, "rotate");
        
        try{
            this->anchor.magneticDeclination = getDouble(anchor, "declination");
        }catch(char const* e){
            std::cerr << "declination is not set because it was not found in the anchor." << std::endl;
            this->anchor.magneticDeclination = std::numeric_limits<double>::quiet_NaN();
        }
        
        latLngConverter_ = std::make_shared<LatLngConverter>(this->anchor);
        
        // Building - change read order to reduce memory usage peak
        //ImageHolder::setMode(ImageHolderMode(heavy));
        BuildingBuilder buildingBuilder;
        
        auto& buildings = getArray(json, "layers");
        
        for(int floor_num = 0; floor_num < buildings.size(); floor_num++) {
            auto& building = buildings.at(floor_num).get<picojson::value::object>();
            auto& param = getObject(building, "param");
            double ppmx = getDouble(param, "ppmx");
            double ppmy = getDouble(param, "ppmy");
            double ppmz = getDouble(param, "ppmz");
            double originx = getDouble(param, "originx");
            double originy = getDouble(param, "originy");
            double originz = getDouble(param, "originz");
            CoordinateSystemParameters coordSysParams(ppmx, ppmy, ppmz, originx, originy, originz);

            auto& data = getString(building, "data");
            std::ostringstream ostr;
            ostr << floor_num << "floor.png";
            
            std::string path = DataUtils::stringToFile(data, workingDir, ostr.str());
            
            int fn = floor_num;
            if (!get(param, "floor").is<picojson::null>()) {
                fn = (int)getDouble(param, "floor");
            }
            
            buildingBuilder.addFloorCoordinateSystemParametersAndImagePath(fn, coordSysParams, path);
            
            msec = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()-s).count();
            std::cerr << "prepare floor model[" << floor_num << "]: " << msec << "ms" << std::endl;
        }        
        msec = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()-s).count();
        std::cerr << "build floor model: " << msec << "ms" << std::endl;
        std::cout << "Create data store" << std::endl << std::endl;
        // Create data store
        dataStore = std::shared_ptr<DataStoreImpl> (new DataStoreImpl());
        dataStore->building(buildingBuilder.build());
        
        deserializedModel = std::shared_ptr<GaussianProcessLDPLMultiModel<State, Beacons>> (new GaussianProcessLDPLMultiModel<State, Beacons>());
        
        bool doTraining = true;
        try{
            try {
                
                if (has(json, "ObservationModelParameters")) {
                    auto& str = getString(json, "ObservationModelParameters");
                    
                    if (/* DISABLES CODE */ (false)) {
                        std::string omppath = DataUtils::stringToFile(str, workingDir, "ObservationModelParameters");
                        
                        msec = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()-s).count();
                        std::cerr << "save deserialized model: " << msec << "ms" << std::endl;
                        
                        std::cerr << omppath << std::endl;
                        //std::istringstream ompss(str);
                        std::ifstream ompss(omppath);
                        //if (ompss) {
                        std::cout << "loading" << std::endl;
                        deserializedModel->load(ompss, false);
                        std::cout << "loaded" << std::endl;
                        //}
                    } else {
                        std::istringstream ompss(str);
                        if (ompss) {
                            std::cout << "loading" << std::endl;
                            deserializedModel->load(ompss, false);
                            std::cout << "loaded" << std::endl;
                        }
                    }
                }
                
                if (has(json, "BinaryObservationModelParameters")) {
                    auto& binaryModelPath = getString(json, "BinaryObservationModelParameters");
                    std::ifstream ifs(workingDir+"/"+binaryModelPath);
                    std::cout << "loading" << std::endl;
                    deserializedModel->load(ifs, true);
                    std::cout << "loaded" << std::endl;
                    msec = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()-s).count();
                    std::cerr << "load deserialized model: " << msec << "ms" << std::endl;
                    doTraining = false;
                }
                
                msec = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()-s).count();
                std::cerr << "load deserialized model: " << msec << "ms" << std::endl;
                doTraining = false;
            } catch(LocException& e){
                throw e;
            } catch(const std::exception& e) {
                LocException ex(std::string(e.what()));
                BOOST_THROW_EXCEPTION(ex);
            } catch(const char* ch){
                //LocException ex((std::string(ch)));
                //BOOST_THROW_EXCEPTION(ex);
            } catch(...){
                BOOST_THROW_EXCEPTION(LocException("..."));
            }
        }catch(LocException& e){
            e << boost::error_info<struct err_info, std::string>("exception at loading ObservationModelParameters");
            throw e;
        }
        
        mLocalizer = std::shared_ptr<StreamParticleFilter>(new StreamParticleFilter());
        if (mFunctionCalledAfterUpdate2 && mUserData) {
            //mLocalizer->updateHandler(mFunctionCalledAfterUpdate2, mUserData);
            mLocalizer->updateHandler(bridgeFunctionCalledAfterUpdate2, mUserDataBridge);
        }
        if (mFunctionCalledAfterUpdate) {
            mLocalizer->updateHandler(mFunctionCalledAfterUpdate);
        }
        userData.localizer = this;
        
        mLocalizer->numStates(nStates);
        mLocalizer->alphaWeaken(alphaWeaken);
        mLocalizer->locationStandardDeviationLowerBound(locLB);
        mLocalizer->optVerbose(isVerboseLocalizer);
        mLocalizer->effectiveSampleSizeThreshold(effectiveSampleSizeThreshold);
        mLocalizer->enablesFloorUpdate(enablesFloorUpdate);
        mLocalizer->dataStore(dataStore);
        
        
        // Sampling data
        
        // Samples samples;
        try{
            auto& samples = getArray(json, "samples");
            for(int i = 0; i < samples.size(); i++) {
                auto& sample = samples.at(i).get<picojson::value::object>();
                auto& data = getString(sample, "data");
                
                //std::string samplepath = DataUtils::stringToFile(data, workingDir);
                //std::ifstream is(samplepath);
                std::istringstream is(data);
                dataStore->readSamples(is);
            }
            {
                std::cerr << dataStore->getSamples().size() << " samples have been loaded" << std::endl;
            }
        }catch(const char* ch){
            std::cerr << "samples have not been loaded." << std::endl;
        }
        msec = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()-s).count();
        std::cerr << "load sample data: " << msec << "ms" << std::endl;
        
        // set unique locations to data store 
        if(dataStore->getSamples().size() != 0){
            const auto& uniLocs = Sample::extractUniqueLocations(dataStore->getSamples());
            dataStore->locations(uniLocs);
        }
        
        
        // set sample locations
        try{
            auto& locationsJarray = getArray(json, "locations");
            Locations locations;
            for(int i = 0; i < locationsJarray.size(); i++) {
                auto& locationsJobj = locationsJarray.at(i).get<picojson::value::object>();
                auto& data = getString(locationsJobj, "data");
                std::istringstream is(data);
                DataUtils::csvLocationsToLocations(is, locations);
            }
            dataStore->locations(locations);
            {
                std::cerr << dataStore->getLocations().size() << " locations have been loaded" << std::endl;
            }
        }catch(const char* ch){
            {
                std::cerr << "locations have not been loaded" << std::endl;
            }
        }
        msec = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()-s).count();
        std::cerr << "load location data: " << msec << "ms" << std::endl;
        
        if(dataStore->getLocations().size()==0){
            BOOST_THROW_EXCEPTION(LocException("Neither samples nor locations have been loaded"));
        }
        
        
        // set BLE beacon locations
        BLEBeacons bleBeacons;
        auto& beacons = getArray(json, "beacons");
        for(int i = 0; i < beacons.size(); i++) {
            auto& beacon = beacons.at(i).get<picojson::value::object>();
            auto& data = getString(beacon, "data");
            
            std::istringstream is(data);
            BLEBeacons bleBeaconsTmp = DataUtils::csvBLEBeaconsToBLEBeacons(is);
            bleBeacons.insert(bleBeacons.end(), bleBeaconsTmp.begin(), bleBeaconsTmp.end());
        }
        dataStore->bleBeacons(bleBeacons);
        msec = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()-s).count();
        std::cerr << "load beacon data: " << msec << "ms" << std::endl;
        
        if(doTraining || forceTraining){
            std::cerr << "Training will be processed" << std::endl;
            // Train observation model
            std::shared_ptr<GaussianProcessLDPLMultiModelTrainer<State, Beacons>>obsModelTrainer( new GaussianProcessLDPLMultiModelTrainer<State, Beacons>());
            obsModelTrainer->setGPType(basicLocalizerOptions.gpType);
            obsModelTrainer->dataStore(dataStore);
            std::shared_ptr<GaussianProcessLDPLMultiModel<State, Beacons>> obsModel( obsModelTrainer->train());
            //localizer->observationModel(obsModel);
            
            std::ostringstream oss;
            obsModel->save(oss, binaryOutput);
            json["ObservationModelParameters"] = (picojson::value)oss.str();
            json.erase("BinaryObservationModelParameters");
            
            std::ofstream of;
            of.open(trainedFile);
            of << v.serialize();
            of.close();
        }
        
        // update additional parameters in the observation model
        deserializedModel->coeffDiffFloorStdev(coeffDiffFloorStdev);
        if(1<=tDelay){
            deserializedModel->tDelay(tDelay);
        }

        // finalize mapdata file
        if(finalizeMapdata){
            std::cerr << "Finalizing map data." << std::endl;
            // convert unique locations to string
            auto uniLocs = dataStore->getLocations();
            std::stringstream ss;
            for(const auto& uloc: uniLocs){
                ss << "0,L," << uloc << std::endl;
            }
            auto uLocLine = ss.str();
            
            // modify json object
            picojson::array locationsArray;
            picojson::object locationsObj;
            locationsObj.insert(std::make_pair("data", picojson::value(uLocLine)));
            locationsArray.push_back(picojson::value(locationsObj));
            json.insert(std::make_pair("locations", picojson::value(locationsArray)));
            json.erase("samples");
            
            if (binaryOutput) {
                std::ofstream of;
                of.open(workingDir+"/"+binaryFile);
                deserializedModel->save(of, binaryOutput);
                of.close();
                json["BinaryObservationModelParameters"] = (picojson::value)binaryFile;
                json.erase("ObservationModelParameters");
            }
            
            // output mapdata
            std::ofstream of;
            of.open(finalizedFile);
            of << v.serialize();
            of.close();
        }
        
        // Instantiate sensor data processors
        // Orientation
        orientationMeterAverageParameters.interval(0.1);
        orientationMeterAverageParameters.windowAveraging(0.1);
        if(orientationMeterType==RAW_AVERAGE){
            orientationMeter = std::make_shared<OrientationMeterAverage>(orientationMeterAverageParameters);
        }else if(orientationMeterType==TRANSFORMED_AVERAGE){
            // TransformedOrientationMeterAverage is not used because drift adjustment may be applied before averaging attitudes.
            orientationMeter = std::make_shared<OrientationMeterAverage>(orientationMeterAverageParameters);
        }else{
            BOOST_THROW_EXCEPTION(LocException("unsupported orientation meter type"));
        }
        
        // Pedometer
        // TODO
        pedometerWSParams.updatePeriod(0.1);
        pedometerWSParams.walkDetectSigmaThreshold(walkDetectSigmaThreshold);
        // END TODO
        pedometer = std::shared_ptr<Pedometer>(new PedometerWalkingState(pedometerWSParams));
        
        // Set dependency
        mLocalizer->orientationMeter(orientationMeter);
        mLocalizer->pedometer(pedometer);
        
        // AltitudeManager
        if(usesAltimeterForFloorTransCheck){
            AltitudeManagerSimple::Ptr altMgr = std::make_shared<AltitudeManagerSimple>();
            altMgr->parameters(altimeterManagerParameters);
            mLocalizer->altitudeManager(altMgr);
        }
        
        // Build System Model
        // TODO (PoseProperty and StateProperty)
        poseProperty->meanVelocity(meanVelocity);
        poseProperty->stdVelocity(stdVelocity);
        poseProperty->diffusionVelocity(diffusionVelocity);
        poseProperty->minVelocity(minVelocity);
        poseProperty->maxVelocity(maxVelocity);
        poseProperty->stdOrientation(stdOrientation/180.0*M_PI);
        poseProperty->stdX(1.0);
        poseProperty->stdY(1.0);
        
        this->meanRssiBias(meanRssiBias_);
        this->minRssiBias(minRssiBias_);
        this->maxRssiBias(maxRssiBias_);
        
        stateProperty->stdRssiBias(stdRssiBias);
        stateProperty->diffusionRssiBias(diffusionRssiBias);
        stateProperty->diffusionOrientationBias(diffusionOrientationBias/180.0*M_PI);
        
        // END TODO
        
        // Build poseRandomWalker
        poseRandomWalker = std::shared_ptr<PoseRandomWalker>(new PoseRandomWalker());
        poseRandomWalkerProperty->orientationMeter(orientationMeter.get());
        poseRandomWalkerProperty->pedometer(pedometer.get());
        poseRandomWalkerProperty->angularVelocityLimit(angularVelocityLimit/180.0*M_PI);
        poseRandomWalkerProperty->doesUpdateWhenStopping(doesUpdateWhenStopping);
        poseRandomWalker->setProperty(poseRandomWalkerProperty);
        poseRandomWalker->setPoseProperty(poseProperty);
        poseRandomWalker->setStateProperty(stateProperty);
        
        // Combine poseRandomWalker and building
        prwBuildingProperty->maxIncidenceAngle(maxIncidenceAngle/180.0*M_PI);
        //prwBuildingProperty.weightDecayRate(0.9);
        //prwBuildingProperty.weightDecayRate(0.96593632892); // this^20 = 0.5
        //prwBuildingProperty.weightDecayRate(0.93303299153); // this^10 = 0.5
        prwBuildingProperty->weightDecayRate(pow(0.5, 1.0/weightDecayHalfLife)); // this^5 = 0.5
        
        prwBuildingProperty->velocityRateFloor(velocityRateFloor);
        prwBuildingProperty->velocityRateElevator(velocityRateElevator);
        prwBuildingProperty->velocityRateStair(velocityRateStair);
        prwBuildingProperty->velocityRateEscalator(velocityRateEscalator);
        prwBuildingProperty->relativeVelocityEscalator(relativeVelocityEscalator);
        
        Building building = dataStore->getBuilding();
        Building::Ptr buildingPtr(new Building(building));
        poseRandomWalkerInBuilding = std::shared_ptr<PoseRandomWalkerInBuilding>(new PoseRandomWalkerInBuilding());
        poseRandomWalkerInBuilding->poseRandomWalker(poseRandomWalker);
        poseRandomWalkerInBuilding->building(buildingPtr);
        poseRandomWalkerInBuilding->poseRandomWalkerInBuildingProperty(prwBuildingProperty);
        
        RandomWalkerProperty::Ptr randomWalkerProperty(new RandomWalkerProperty);
        randomWalkerProperty->sigma = 0.25;
        randomWalker.reset(new RandomWalker<State, SystemModelInput>());
        randomWalker->setProperty(randomWalkerProperty);

        
        // Setup RandomWalkerMotion
        RandomWalkerMotionProperty::Ptr randomWalkerMotionProperty(new RandomWalkerMotionProperty);
        randomWalkerMotionProperty->pedometer(pedometer);
        randomWalkerMotionProperty->orientationMeter(orientationMeter);
        randomWalkerMotionProperty->usesAngularVelocityLimit(true);
        randomWalkerMotionProperty->angularVelocityLimit(angularVelocityLimit/180.0*M_PI);
        randomWalkerMotionProperty->sigmaStop = sigmaStop;
        randomWalkerMotionProperty->sigmaMove = sigmaMove;
        
        if (localizeMode == RANDOM_WALK_ACC_ATT) {
            mLocalizer->systemModel(poseRandomWalkerInBuilding);
        }else if(localizeMode == RANDOM_WALK_ACC){
            RandomWalkerMotion<State, SystemModelInput>::Ptr randomWalkerMotion(new RandomWalkerMotion<State, SystemModelInput>);
            randomWalkerMotion->setProperty(randomWalkerMotionProperty);
            // Setup SystemModelInBuilding
            SystemModelInBuilding<State, SystemModelInput>::Ptr rwMotionBldg(new SystemModelInBuilding<State, SystemModelInput>(randomWalkerMotion, buildingPtr, prwBuildingProperty) );
            mLocalizer->systemModel(rwMotionBldg);
        }
        else if (localizeMode == RANDOM_WALK) {
            mLocalizer->systemModel(randomWalker);
        }
        else if (localizeMode == WEAK_POSE_RANDOM_WALKER){
            WeakPoseRandomWalker<State, SystemModelInput>::Ptr wPRW(new WeakPoseRandomWalker<State, SystemModelInput>);
            wPRW->setProperty(randomWalkerMotionProperty);
            wPRW->setPoseProperty(poseProperty);
            wPRW->setStateProperty(stateProperty);
            
            wPRWproperty->probabilityOrientationBiasJump(probabilityOrientationBiasJump);
            wPRWproperty->probabilityBackwardMove(probabilityBackwardMove);
            wPRWproperty->probabilityOrientationJump(0.0);
            wPRWproperty->poseRandomWalkRate(poseRandomWalkRate);
            wPRWproperty->randomWalkRate(randomWalkRate);
            wPRW->setWeakPoseRandomWalkerProperty(wPRWproperty);
            SystemModelInBuilding<State, SystemModelInput>::Ptr wPRWBldg(new SystemModelInBuilding<State, SystemModelInput>(wPRW, buildingPtr, prwBuildingProperty) );
            mLocalizer->systemModel(wPRWBldg);
        }
        
        // set resampler
        resampler = std::shared_ptr<Resampler<State>>(new GridResampler<State>());
        mLocalizer->resampler(resampler);
        // Set status initializer
        ////PoseProperty poseProperty;
        ////StateProperty stateProperty;
        
        statusInitializer = std::shared_ptr<StatusInitializerImpl>(new StatusInitializerImpl());
        statusInitializer->dataStore(dataStore)
        .poseProperty(poseProperty).stateProperty(stateProperty);
        mLocalizer->statusInitializer(statusInitializer);
        
        // Set localizer
        mLocalizer->observationModel(deserializedModel);
        
        // Beacon filter
        auto registeredBeaconFilter = std::make_shared<RegisteredBeaconFilter>(bleBeacons);
        auto strongestBeaconFilter = std::make_shared<StrongestBeaconFilter>(nStrongest);
        auto filterChain = std::make_shared<BeaconFilterChain>();
        filterChain->addFilter(registeredBeaconFilter).addFilter(strongestBeaconFilter);
        beaconFilter = filterChain;
        mLocalizer->beaconFilter(filterChain);
        
        // Set standard deviation of Pose
        double stdevX = 0.25;
        double stdevY = 0.25;
        double orientation = 10*M_PI;
        stdevPose.x(stdevX).y(stdevY).orientation(orientation);
        
        // ObservationDependentInitializer
        obsDepInitializer = std::shared_ptr<MetropolisSampler<State, Beacons>>(new MetropolisSampler<State, Beacons>());
        
        obsDepInitializer->observationModel(deserializedModel);
        obsDepInitializer->statusInitializer(statusInitializer);
        msParams.burnIn = nBurnIn;
        msParams.radius2D = burnInRadius2D; // 10[m]
        msParams.interval = burnInInterval;
        msParams.withOrdering = true;
        msParams.initType = burnInInitType;

        obsDepInitializer->parameters(msParams);
        obsDepInitializer->isVerbose = isVerboseLocalizer;
        mLocalizer->observationDependentInitializer(obsDepInitializer);
        
        // Mixture settings
        // double mixProba = 0.001;

        StreamParticleFilter::MixtureParameters mixParams;
        mixParams.mixtureProbability = mixProba;
        mixParams.rejectDistance = rejectDistance;
        mixParams.rejectFloorDifference(rejectFloorDifference);
        mixParams.nBeaconsMinimum = nBeaconsMinimum;
        mLocalizer->mixtureParameters(mixParams);
        
        mLocalizer->floorTransitionParameters(pfFloorTransParams);
        mLocalizer->locationStatusMonitorParameters(locationStatusMonitorParameters);
        
        msec = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()-s).count();
        std::cerr << "finish setModel: " << msec << "ms" << std::endl;
        isReady = true;
        return *this;
    }
    
    double BasicLocalizer::estimatedRssiBias() {
        return mEstimatedRssiBias;
    }
    
    void BasicLocalizer::updateStateProperty() {
        if (poseRandomWalker) {
            poseRandomWalker->setStateProperty(stateProperty);
            poseRandomWalker->setProperty(poseRandomWalkerProperty);
        }
        if (statusInitializer) {
            statusInitializer->stateProperty(stateProperty);
        }
    }
    
    void BasicLocalizer::meanRssiBias(double b) {
        meanRssiBias_ = b;
        stateProperty->meanRssiBias(b);
        //updateStateProperty();
    }

    void BasicLocalizer::minRssiBias(double b) {
        minRssiBias_ = b;
        stateProperty->minRssiBias(b);
        //updateStateProperty();
    }
    void BasicLocalizer::maxRssiBias(double b) {
        maxRssiBias_ = b;
        stateProperty->maxRssiBias(b);
        //updateStateProperty();
    }
    
    /*
    void BasicLocalizer::angularVelocityLimit(double a) {
        poseRandomWalkerProperty.angularVelocityLimit(a);
        updateStateProperty();
    }
    */

    void BasicLocalizer::normalFunction(NormalFunction type, double option) {
        if (type == NORMAL) {
            deserializedModel->normFunc = MathUtils::logProbaNormal;
        }
        else if (type == TDIST) {
            deserializedModel->normFunc = MathUtils::logProbatDistFunc(option);
        }
    }
    
    // for orientation initialization
    void BasicLocalizer::headingConfidenceForOrientationInit(double confidence){
        if(!(0.0<=confidence && confidence<=1.0)){
            BOOST_THROW_EXCEPTION(LocException("range check error (0.0<=confidence && confidence<=1.0)"));
        }
        headingConfidenceForOrientationInit_ = confidence;
    }

    LatLngConverter::Ptr BasicLocalizer::latLngConverter(){
        return latLngConverter_;
    }
    
    void BasicLocalizer::disableAcceleration(bool disable, long timestamp){
        if(mFunctionCalledToLog && mDisableAcceleration!=disable){
            std::string str = "DisableAcceleration,"+std::to_string(disable)+","+std::to_string(timestamp);
            mFunctionCalledToLog(mUserDataToLog, str);
        }
        mDisableAcceleration = disable;
    }
    
    //LocalHeadingBuffer
    LocalHeadingBuffer::LocalHeadingBuffer(size_t n){
        buffer_ = boost::circular_buffer<LocalHeading>(n);
    }
    LocalHeadingBuffer::LocalHeadingBuffer(const LocalHeadingBuffer& lhb){
        buffer_ = boost::circular_buffer<LocalHeading>(lhb.buffer_);
    }
    LocalHeadingBuffer& LocalHeadingBuffer::operator=(const LocalHeadingBuffer& lhb){
        this->buffer_ = boost::circular_buffer<LocalHeading>(lhb.buffer_);
        return *this;
    }
    void LocalHeadingBuffer::push_back(const LocalHeading& lh){
        std::lock_guard<std::mutex> lock(mtx_);
        buffer_.push_back(lh);
    }
    LocalHeading& LocalHeadingBuffer::back(){
        std::lock_guard<std::mutex> lock(mtx_);
        return buffer_.back();
    }
    size_t LocalHeadingBuffer::size(){
        return buffer_.size();
    }
    
}
