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

#ifndef OrientationMeterAverage_hpp
#define OrientationMeterAverage_hpp

#include <stdio.h>
#include <cmath>

#include "bleloc.h"
#include "OrientationMeter.hpp"

namespace loc{
    
    class OrientationMeterAverageParameters{
        double interval_ = 0.0;
        double windowAveraging_ = 0.1; // weighted averaging time. 0.1[s]
        
    public:
        OrientationMeterAverageParameters& interval(double interval){
            this->interval_ = interval;
            return *this;
        }
        
        OrientationMeterAverageParameters& windowAveraging(double windowAveraging){
            this->windowAveraging_ = windowAveraging;
            return *this;
        }
        
        double interval() const{
            return interval_;
        }
        
        double windowAveraging() const{
            return windowAveraging_;
        }
        
    };
    
    class OrientationMeterAverage : public OrientationMeter{
    
    private:
        long prevTimestamp = 0;
        double theta = 0;
        bool isUpdated_ = false;
        long count = 1;
        
        OrientationMeterAverageParameters parameters_;
        
        double getRatio();
        double getRatio(double dt);
        
    public:
        OrientationMeterAverage(){}
        OrientationMeterAverage(OrientationMeterAverageParameters parameters){
            parameters_ = parameters;
        }
        
        ~OrientationMeterAverage(){}
        
        OrientationMeter& putAttitude(Attitude attitude) override;
        bool isUpdated() override;
        double getYaw() override;
        void reset() override;
    };
}


#endif /* OrientationMeterAverage_hpp */
