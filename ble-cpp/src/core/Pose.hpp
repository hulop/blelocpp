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

#ifndef Pose_hpp
#define Pose_hpp

#include <stdio.h>
#include <vector>
#include <cmath>
#include "Location.hpp"

namespace loc {
    
    class Pose;
    using Poses = std::vector<Pose> ;
    
    
    class Pose : public Location{
        
    private:
        //Location mLocation;
        double mOrientation = 0;
        double mVelocity = 0;
        double mNormalVelocity = 0;
        
    public:
        Pose() = default;
        ~Pose() = default;
        
        //Pose(const Pose& pose) = default;
        Pose(const Location& location) : Location(location){}
        
        //Location location();
        
        double x() const;
        double y() const;
        double floor() const;
        double z() const;
        
         
        double orientation() const;
        double velocity() const;
        double normalVelocity() const;
        
        double vx() const;
        double vy() const;
        
        //Pose* location(Location);
        
        Pose& x(double);
        Pose& y(double);
        Pose& z(double);
        Pose& floor(double);
        
        Pose& orientation(double);
        Pose& velocity(double);
        Pose& normalVelocity(double);
        
        template<class Tpose>
        static Pose mean(std::vector<Tpose> poses);
        
        static double normalizeOrientaion(double orientation);
        static double computeOrientationDifference(double o1, double o2);
        
        // for string stream
        friend std::ostream& operator<<(std::ostream&os, const Pose& pose);
    };
    
    // Template function
    template<class Tpose>
    Pose Pose::mean(std::vector<Tpose> poses){
        size_t n = poses.size();
        double xm = 0, ym = 0, zm = 0, floorm = 0;
        double vxm = 0, vym = 0;
        double vxrepm = 0, vyrepm = 0;
        
        for(Tpose pose: poses){
            xm += pose.Location::x();
            ym += pose.Location::y();
            zm += pose.Location::z();
            floorm += pose.Location::floor();
            vxm += pose.velocity()*std::cos(pose.orientation());
            vym += pose.velocity()*std::sin(pose.orientation());
            vxrepm += pose.normalVelocity()*cos(pose.orientation());
            vyrepm += pose.normalVelocity()*sin(pose.orientation());
        }
        xm/=n;
        ym/=n;
        zm/=n;
        floorm/=n;
        
        vxm/=n;
        vym/=n;
        double vm = std::sqrt(vxm*vxm + vym*vym);
        
        vxrepm/=n;
        vyrepm/=n;
        double vrepm = std::sqrt(vxrepm*vxrepm + vyrepm*vyrepm);
        
        double orientationm = atan2(vyrepm, vxrepm); // orientation must be calculated by representative velocity.
        
        Pose poseMean;
        poseMean.x(xm).y(ym).z(zm).floor(floorm);
        poseMean.orientation(orientationm).velocity(vm).normalVelocity(vrepm);
        
        return poseMean;
    }
    
    
    class PoseProperty{
        double meanVelocity_ = 1.0; // [m/s]
        double stdVelocity_ = 0.25; // [m/s]
        //double driftVelocity = 0.05; // [m/s/s]
        double diffusionVelocity_ = 0.05; // [m/s/s]
        double minVelocity_ = 0.1; // [m/s]
        double maxVelocity_ = 1.5; // [m/s]
        
        double stdOrientation_ = 3.0/180*M_PI; // [radian/s]
        
    public:
        PoseProperty& meanVelocity(double meanVelocity);
        double meanVelocity() const;
        PoseProperty& stdVelocity(double stdVelocity);
        double stdVelocity() const;
        PoseProperty& diffusionVelocity(double diffusionVelocity);
        double diffusionVelocity() const;
        PoseProperty& minVelocity(double minVelocity);
        double minVelocity() const;
        PoseProperty& maxVelocity(double maxVelocity);
        double maxVelocity() const;
        PoseProperty& stdOrientation(double stdOrientation);
        double stdOrientation() const;
    };
}

#endif /* Pose_hpp */
