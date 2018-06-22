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

#ifndef StreamLocalizer_hpp
#define StreamLocalizer_hpp

#include <stdio.h>
#include "bleloc.h"
#include "Altimeter.hpp"
#include "Heading.hpp"

namespace loc {
    class StreamLocalizer{
    public:
        virtual ~StreamLocalizer(){};
        
        virtual StreamLocalizer& updateHandler(void (*functionCalledAfterUpdate)(Status*)) = 0;
        virtual StreamLocalizer& updateHandler(void (*functionCalledAfterUpdate)(void*, Status*), void* inUserData) = 0;
        
        virtual StreamLocalizer& putAcceleration(const Acceleration acceleration) = 0;
        virtual StreamLocalizer& putAttitude(const Attitude attitude) = 0;
        virtual StreamLocalizer& putBeacons(const Beacons beacons) = 0;
        virtual StreamLocalizer& putLocalHeading(const LocalHeading heading) = 0;
        virtual StreamLocalizer& putAltimeter(const Altimeter altimeter) = 0;
        virtual StreamLocalizer& putImageLocalizedPose(long timestamp, const Pose pose) = 0;
        virtual Status* getStatus() = 0;
        
        virtual bool resetStatus() = 0;
        virtual bool resetStatus(Pose pose) = 0;
        virtual bool resetStatus(Pose meanPose, Pose stdevPose) = 0;
        virtual bool resetStatus(const Beacons& beacons) = 0;
        virtual bool resetStatus(const Location& location, const Beacons& beacons) = 0;
        
        virtual bool resetStatus(Pose meanPose, Pose stdevPose, double rateContami) = 0;
    };    
}

#endif /* StreamLocalizer_hpp */
