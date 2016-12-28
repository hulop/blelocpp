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

#include "Heading.hpp"

namespace loc{
    
    Heading::Heading(long timestamp, double magneticHeading, double trueHeading, double headingAccuracy){
        timestamp_ = timestamp;
        magneticHeading_ = magneticHeading;
        trueHeading_ = trueHeading;
        headingAccuracy_ = headingAccuracy;
    }
    
    Heading::Heading(long timestamp_, double magneticHeading_, double trueHeading_, double headingAccuracy_, double x_, double y_, double z_){
        timestamp(timestamp_);
        magneticHeading(magneticHeading_);
        trueHeading(trueHeading_);
        headingAccuracy(headingAccuracy_);
        x(x_);
        y(y_);
        z(z_);
    }
    
    void Heading::timestamp(long timestamp_){
        this->timestamp_ = timestamp_;
    }
    long Heading::timestamp() const{
        return timestamp_;
    }
    void Heading::magneticHeading(double magneticHeading_){
        this->magneticHeading_ = magneticHeading_;
    }
    double Heading::magneticHeading() const{
        return magneticHeading_;
    }
    void Heading::trueHeading(double trueHeading_){
        this->trueHeading_ = trueHeading_;
    }
    double Heading::trueHeading() const{
        return trueHeading_;
    }
    void Heading::headingAccuracy(double headingAccuracy_){
        this->headingAccuracy_ = headingAccuracy_;
    }
    double Heading::headingAccuracy() const{
        return headingAccuracy_;
    }
    
    void Heading::x(double x_){
        this->x_ = x_;
    }
    double Heading::x() const{
        return x_;
    }
    void Heading::y(double y_){
        this->y_ = y_;
    }
    double Heading::y() const{
        return y_;
    }
    void Heading::z(double z_){
        this->z_ = z_;
    }
    double Heading::z() const{
        return z_;
    }
    
    
    
    // LocalHeading
    LocalHeading::LocalHeading(long timestamp, double orientation, double orientationDeviation){
        timestamp_ = timestamp;
        orientation_ = orientation;
        orientationDeviation_ = orientationDeviation;
    }
    
    LocalHeading& LocalHeading::timestamp(long timestamp){
        timestamp_ = timestamp;
        return *this;
    }
    
    LocalHeading& LocalHeading::orientation(double orientation){
        orientation_ = orientation;
        return *this;
    }
    
    LocalHeading& LocalHeading::orientationDeviation(double orientationDeviation){
        orientationDeviation_ = orientationDeviation;
        return *this;
    }
    
    long LocalHeading::timestamp() const{
        return timestamp_;
    }
    
    double LocalHeading::orientation() const{
        return orientation_;
    }
    
    double LocalHeading::orientationDeviation() const{
        return orientationDeviation_;
    }
}
