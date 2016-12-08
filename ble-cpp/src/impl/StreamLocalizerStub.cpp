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

#include "bleloc.h"
#include "StreamLocalizer.hpp"
#include "StreamLocalizerStub.hpp"

namespace loc{
    
    StreamLocalizerStub::StreamLocalizerStub(){}
    StreamLocalizerStub::~StreamLocalizerStub(){}

    void StreamLocalizerStub::callback(loc::Status * status){
        if(mFunctionCalledAfterUpdate!=NULL){
            mFunctionCalledAfterUpdate(status);
        }
        if(mFunctionCalledAfterUpdate2!=NULL){
            mFunctionCalledAfterUpdate2(mUserData, status);
        }
    }
    
    StreamLocalizer& StreamLocalizerStub::putAcceleration(const Acceleration acceleration){
        updateStateStub();
        callback(status);
        return *this;
    }
    StreamLocalizer& StreamLocalizerStub::putAttitude(const Attitude attitude) {
        return *this;
    }
    StreamLocalizer& StreamLocalizerStub::putBeacons(const Beacons beacons) {
        updateStateStub();
        callback(status);
        return *this;
    }
    StreamLocalizer& StreamLocalizerStub::putHeading(const Heading heading) {
        return *this;
    }
    StreamLocalizer& StreamLocalizerStub::putAltimeter(const Altimeter altimeter) {
        return *this;
    }
    
    void StreamLocalizerStub::updateStateStub(){
        Status* statusNew = new Status();
        auto loc = std::shared_ptr<Location>(new Location(1,2,0,0));
        auto pose = std::shared_ptr<Pose>(new Pose);
        pose->x(loc->x()).y(loc->y()).z(loc->z());
        statusNew->meanLocation(loc);
        statusNew->meanPose(pose);
        status = statusNew;
    }

    StreamLocalizer& StreamLocalizerStub::updateHandler(void (*funcCalledAfterUpdate)(Status*)) {
        mFunctionCalledAfterUpdate = funcCalledAfterUpdate;
        return *this;
    }
    
    StreamLocalizer& StreamLocalizerStub::updateHandler(void (*funcCalledAfterUpdate)(void*, Status*), void* inUserData) {
        mFunctionCalledAfterUpdate2 = funcCalledAfterUpdate;
        mUserData = inUserData;
        return *this;
    }
    
    Status* StreamLocalizerStub::getStatus() {
        return status;
    }
    
    bool StreamLocalizerStub::resetStatus(){
        std::cout << "StreamLocalizer::resetStatus() is not supported." <<std::endl;
        return true;
    }
    
    bool StreamLocalizerStub::resetStatus(Pose pose){
        resetStatus();
        return true;
    }
    
    bool StreamLocalizerStub::resetStatus(Pose meanPose, Pose stdevPose){
        resetStatus();
        return true;
    }
    
    bool StreamLocalizerStub::resetStatus(Pose meanPose, Pose stdevPose, double contamiRate){
        resetStatus();
        return true;
    }
    
    bool StreamLocalizerStub::resetStatus(const loc::Beacons &beacons){
        resetStatus();
        return true;
    }
    bool StreamLocalizerStub::resetStatus(const loc::Location &location, const loc::Beacons &beacons){
        resetStatus();
        return true;
    }
    
    
    /*
    StreamLocalizer* StreamLocalizerStub::pedometer(Pedometer* pedometer){
        return this;
    }
    StreamLocalizer* StreamLocalizerStub::orientationMeter(OrientationMeter* orientationMeter){
        return this;
    }
    */

}
