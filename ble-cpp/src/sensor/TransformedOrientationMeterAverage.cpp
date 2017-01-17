/*******************************************************************************
 * Copyright (c) 2014, 2017  IBM Corporation and others
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

#include "TransformedOrientationMeterAverage.hpp"

namespace loc{
    
    class RotationMatrix{
    public:
        double m11;
        double m12;
        double m13;
        double m21;
        double m22;
        double m23;
        double m31;
        double m32;
        double m33;
        
        RotationMatrix(double roll, double pitch, double yaw){
            double c3 = std::cos(roll);
            double s3 = std::sin(roll);
            double c2 = std::cos(pitch);
            double s2 = std::sin(pitch);
            double c1 = std::cos(yaw);
            double s1 = std::sin(yaw);
            
            m11 = c1*c3 - s1*s2*s3;
            m12 = -c2*s1;
            m13 = c1*s3 + c3*s1*s2;
            m21 = c3*s1 + c1*s2*s3;
            m22 = c1*c2;
            m23 = s1*s3 - c1*c3*s2;
            m31 = -c2*s3;
            m32 = s2;
            m33 = c2*c3;
        }
        
        ~RotationMatrix() = default;
    };
    
    double TransformedOrientationMeterAverage::getRatio(){
        double ratio_tmp = 1.0/count;
        double ratio = fmax(ratio_tmp, parameters_.windowAveraging());
        return ratio;
    }
    
    double TransformedOrientationMeterAverage::getRatio(double dt){
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
    
    double TransformedOrientationMeterAverage::transformOrientation(const Attitude& attitude) const{
        RotationMatrix R(attitude.roll(),attitude.pitch(),attitude.yaw());
        double a0 = std::sqrt(R.m12*R.m12 + R.m22*R.m22);
        double a1 = R.m32;
        
        double Y_x = a0*R.m12 - a1*R.m13;
        double Y_y = a0*R.m22 - a1*R.m23;
        
        double ori = std::atan2(-Y_x, Y_y);
        
        return ori;
    }
    
    
    OrientationMeter& TransformedOrientationMeterAverage::putAttitude(Attitude attitude){
        
        long timestamp = attitude.timestamp();
        
        double thetaNew =  this->transformOrientation(attitude);
        
        if(prevTimestamp==0){
            prevTimestamp = timestamp;
            theta = thetaNew;
        }
        if(timestamp < prevTimestamp){
            std::cout << "Inconsistent timestamp was found in OrientationMeter. OrientationMeter status was resetted." << std::endl;
            this->reset();
        }
        
        double dt = (timestamp - prevTimestamp)/1000.0;
        
        double x = std::cos(theta);
        double y = std::sin(theta);
        
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
    
    bool TransformedOrientationMeterAverage::isUpdated(){
        return isUpdated_;
    }
    
    double TransformedOrientationMeterAverage::getYaw(){
        return theta;
    }
    
    void TransformedOrientationMeterAverage::reset(){
        prevTimestamp = 0;
        isUpdated_ = false;
        count = 1;
    }
    
}
