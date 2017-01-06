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

#include "Status.hpp"
#include "Location.hpp"

namespace loc{
    
    Status::Status() : states_(new std::vector<State>()){
    }
    
    Status::~Status(){}
    
    Status::Status(const Status& status){
        step_ = status.step_;
        locationStatus_ = status.locationStatus_;
        timestamp_ = status.timestamp_;
        mWasFloorUpdated = status.mWasFloorUpdated;
        auto meanLoc = status.meanLocation();
        auto meanPose = status.meanPose();
        auto states = status.states();
        if(meanLoc){
            meanLocation_ = Location::Ptr(new Location(*meanLoc));
        }
        if(meanPose){
            meanPose_ = Pose::Ptr(new Pose(*meanPose));
        }
        if(states){
            states_ = std::shared_ptr<States>(new States(*states));
        }
    }
    
    Status& Status::operator=(const Status& status){
        step_ = status.step_;
        locationStatus_ = status.locationStatus_;
        timestamp_ = status.timestamp_;
        mWasFloorUpdated = status.mWasFloorUpdated;
        auto meanLoc = status.meanLocation();
        auto meanPose = status.meanPose();
        auto states = status.states();
        if(meanLoc){
            meanLocation_ = Location::Ptr(new Location(*meanLoc));
        }
        if(meanPose){
            meanPose_ = Pose::Ptr(new Pose(*meanPose));
        }
        if(states){
            states_ = std::shared_ptr<States>(new States(*states));
        }
        return *this;
    }
    
    std::shared_ptr<Location> Status::meanLocation() const{
        return meanLocation_;
    }
    
    std::shared_ptr<Pose> Status::meanPose() const{
        return meanPose_;
    }
    
    long Status::timestamp() const{
        return timestamp_;
    }
    
    std::shared_ptr<std::vector<State>> Status::states() const{
        return states_;
    }
    
    Status& Status::meanLocation(std::shared_ptr<Location> location){
        meanLocation_ = location;
        return *this;
    }
    
    Status& Status::meanPose(std::shared_ptr<Pose> pose){
        meanPose_ = pose;
        return *this;
    }
    
    Status& Status::timestamp(long timestamp){
        timestamp_ = timestamp;
        return *this;
    }
    
    Status& Status::states(std::vector<State>* states){
        auto statesTmp = std::shared_ptr<States>();
        statesTmp.reset(states);
        return this->states(statesTmp);
    }
    
    Status& Status::states(std::shared_ptr<std::vector<State>> states){
        this->step(Status::OTHER);
        
        states_ = states;
        size_t n = states->size();
        std::vector<double> weights(n);
        for(int i=0; i<n; i++){
            weights[i] = states->at(i).weight();
        }
        // Compute mean Location
        auto meanLoc = std::shared_ptr<Location>(new Location(Location::weightedMean(*states, weights)));
        meanLocation(meanLoc);
        auto meanPs = std::shared_ptr<Pose>(new Pose(Pose::weightedMean(*states, weights)));
        meanPose(meanPs);
        return *this;
    }
    
    Status& Status::states(std::shared_ptr<std::vector<State>> states, Step step){
        this->states(states);
        this->step(step);
        return *this;
    }
    
    Status::Step Status::step() const{
        return step_;
    }
    
    Status::LocationStatus Status::locationStatus() const{
        return locationStatus_;
    }
    
    Status& Status::step(Status::Step step){
        step_ = step;
        
        if(step_==FILTERING_WITH_RESAMPLING || step_==FILTERING_WITHOUT_RESAMPLING || step_==RESET){
            mWasFloorUpdated =  true;
        }else{
            mWasFloorUpdated = false;
        }
        
        return *this;
    }
    
    Status& Status::locationStatus(Status::LocationStatus locationStatus){
        locationStatus_ = locationStatus;
        return *this;
    }
    
    
    bool Status::wasFloorUpdated() const{
        return mWasFloorUpdated;
    }
    
    void Status::wasFloorUpdated(bool wasFloorUpdated){
        mWasFloorUpdated = wasFloorUpdated;
    }
    
    std::string Status::locationStatusToString(const loc::Status::LocationStatus & locStatus){
        std::string str;
        switch(locStatus){
            case(Status::UNKNOWN):
                str =  "UNKNOWN";
                break;
            case(Status::LOCATING):
                str = "LOCATING";
                break;
            case(Status::STABLE):
                str = "STABLE";
                break;
            case(Status::UNSTABLE):
                str = "UNSTABLE";
                break;
            case(Status::NIL):
                str = "NIL";
                break;
        }
        return str;
    }
    
}
