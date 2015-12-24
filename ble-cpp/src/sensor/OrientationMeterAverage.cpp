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

#include "OrientationMeterAverage.hpp"

namespace loc{
    
    double OrientationMeterAverage::getRatio(){
        double ratio_tmp = 1.0/count;
        double ratio = fmax(ratio_tmp, parameters_.windowAveraging());
        return ratio;
    }
    
    double OrientationMeterAverage::getRatio(double dt){
        if(dt==0){
            return 1.0;
        }
        if(parameters_.interval() != 0.0){
            dt = parameters_.interval();
        }
        int n = static_cast<int>(parameters_.windowAveraging()/dt);
        n = n==0? 1 :n;
        double ratio = fmax(1.0/count, 1.0/n);
        
        if(count>=n){
            isUpdated_ = true;
        }else{
            isUpdated_ = false;
        }
        
        return ratio;
    }
    

    OrientationMeter& OrientationMeterAverage::putAttitude(Attitude attitude){
        
        long timestamp = attitude.timestamp();
        
        if(prevTimestamp==0){
            prevTimestamp = timestamp;
            theta = attitude.yaw();
        }
        if(timestamp < prevTimestamp){
            std::cout << "Inconsistent timestamp was found in OrientationMeter. OrientationMeter status was resetted." << std::endl;
            this->reset();
        }
        
        double dt = (timestamp - prevTimestamp)/1000.0;
        
        double x = std::cos(theta);
        double y = std::sin(theta);
        
        double thetaNew = attitude.yaw();
        double xNew = std::cos(thetaNew);
        double yNew = std::sin(thetaNew);
        
        double ratio = getRatio(dt);
        
        double xWA = (1.0-ratio) * x + ratio * xNew;
        double yWA = (1.0-ratio) * y + ratio * yNew;
        
        theta = std::atan2(yWA, xWA);
        count ++ ;
        
        prevTimestamp = timestamp;
        return *this;
    }
    
    bool OrientationMeterAverage::isUpdated(){
        return isUpdated_;
    }
    
    double OrientationMeterAverage::getYaw(){
        return theta;
    }
    
    void OrientationMeterAverage::reset(){
        prevTimestamp = 0;
        isUpdated_ = false;
        count = 1;
    }
    
    
}
