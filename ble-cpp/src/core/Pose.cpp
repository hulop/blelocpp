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


#include "Pose.hpp"
#include <cmath>

namespace loc{

    /*
    Location Pose::location(){
        return mLocation;
    }
    */
    
    double Pose::x() const{
        return Location::x();
    }
    double Pose::y() const{
        return Location::y();
    }
    double Pose::floor() const{
        return Location::floor();
    }
    double Pose::z() const{
        return Location::z();
    }

    double Pose::orientation() const{
      return mOrientation;
    }
    double Pose::velocity() const{
        return mVelocity;
    }
    double Pose::normalVelocity() const{
        return mNormalVelocity;
    }

    double Pose::vx() const{
        return velocity()*cos(orientation());
    }
    
    double Pose::vy() const{
        return velocity()*sin(orientation());
    }
    
    /*
    Pose& Pose::location(Location location){
        mLocation = location;
        return *this;
    }
    */
    Pose& Pose::x(double x){
        Location::x(x);
        //mLocation.x(x);
        return *this;
    }
    Pose& Pose::y(double y){
        Location::y(y);
        //mLocation.y(y);
        return *this;
    }
    Pose& Pose::z(double z){
        Location::z(z);
        //mLocation.z(z);
        return *this;
    }
    Pose& Pose::floor(double floor){
        Location::floor(floor);
        //mLocation.floor(floor);
        return *this;
    }

    Pose& Pose::orientation(double orientation){
        mOrientation = orientation;
        return *this;
    }

    Pose& Pose::velocity(double velocity){
        mVelocity =  velocity;
        return *this;
    }
    Pose& Pose::normalVelocity(double normalVelocity){
        mNormalVelocity = normalVelocity;
        return *this;
    }
    
    double Pose::normalizeOrientaion(double orientation){
        double x = std::cos(orientation);
        double y = std::sin(orientation);
        return std::atan2(y,x);
    }

    
    double Pose::computeOrientationDifference(double o1, double o2){
        double ret = o2-o1;
        if (ret > M_PI) {
            ret -= M_PI*2;
        }
        if (ret < -M_PI) {
            ret += M_PI*2;
        }
        return ret;
    }
    
    
    // for string stream
    std::ostream& operator<<(std::ostream&os, const Pose& pose){
        os << pose.x() <<"," << pose.y() <<"," << pose.z() <<"," << pose.floor()
        << "," << pose.orientation() << "," << pose.velocity() <<"," << pose.normalVelocity();
        return os;
    }
    
    
    // PoseProperty
    PoseProperty& PoseProperty::meanVelocity(double meanVelocity){
        this->meanVelocity_ = meanVelocity;
        return *this;
    }
    
    double PoseProperty::meanVelocity() const{
        return meanVelocity_;
    }
    
    PoseProperty& PoseProperty::stdVelocity(double stdVelocity){
        stdVelocity_ = stdVelocity;
        return *this;
    }
    
    double PoseProperty::stdVelocity() const{
        return stdVelocity_;
    }
    
    PoseProperty& PoseProperty::diffusionVelocity(double diffusionVelocity){
        diffusionVelocity_ = diffusionVelocity;
        return *this;
    }
    
    double PoseProperty::diffusionVelocity() const{
        return diffusionVelocity_;
    }
    
    PoseProperty& PoseProperty::minVelocity(double minVelocity){
        minVelocity_ = minVelocity;
        return *this;
    }
    
    double PoseProperty::minVelocity() const{
        return minVelocity_;
    }
    
    PoseProperty& PoseProperty::maxVelocity(double maxVelocity){
        maxVelocity_ = maxVelocity;
        return *this;
    }
    
    double PoseProperty::maxVelocity() const{
        return maxVelocity_;
    }
    
    PoseProperty& PoseProperty::stdOrientation(double stdOrientation){
        stdOrientation_ = stdOrientation;
        return *this;
    }
    
    double PoseProperty::stdOrientation() const{
        return stdOrientation_;
    }
    
    
}
