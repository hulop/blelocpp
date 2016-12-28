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

#ifndef StreamLocalizerStub_hpp
#define StreamLocalizerStub_hpp

#include <stdio.h>
#include "bleloc.h"
#include "StreamLocalizer.hpp"

namespace loc {
    class StreamLocalizerStub : public StreamLocalizer{
    public:
        
        StreamLocalizerStub();
        ~StreamLocalizerStub();

        StreamLocalizer& putAcceleration(const Acceleration acceleration) override;
        StreamLocalizer& putAttitude(const Attitude attitude) override;
        StreamLocalizer& putBeacons(const Beacons beacons) override;
        virtual StreamLocalizer& putLocalHeading(const LocalHeading heading) override;
        virtual StreamLocalizer& putAltimeter(const Altimeter altimeter) override;
        StreamLocalizer& updateHandler(void (*functionCalledAfterUpdate)(Status*)) override;
        StreamLocalizer& updateHandler(void (*functionCalledAfterUpdate)(void*, Status*), void* inUserData) override;
        
        Status* getStatus() override;
        
        bool resetStatus() override;
        bool resetStatus(Pose pose) override;
        bool resetStatus(Pose meanPose, Pose stdevPose) override;
        bool resetStatus(Pose meanPose, Pose stdevPose, double contamiRate) override;
        /*
        StreamLocalizer* pedometer(Pedometer* pedometer);
        StreamLocalizer* orientationMeter(OrientationMeter*  orientationMeter);
        */
        bool resetStatus(const Beacons& beacons) override;
        bool resetStatus(const Location& location, const Beacons& beacons) override;
        
    private:
        Status* status;
        void updateStateStub();
        
        void callback(Status*);
        void (*mFunctionCalledAfterUpdate)(Status*) = NULL;
        void (*mFunctionCalledAfterUpdate2)(void*, Status*) = NULL;
        void *mUserData;
    };

}

#endif /* StreamLocalizerStub_hpp */
