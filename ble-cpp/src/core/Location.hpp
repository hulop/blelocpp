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

#ifndef Location_hpp
#define Location_hpp

#include <iostream>
#include <vector>
#include <cmath>

#include "LatLngUtil.hpp"

namespace loc{
    
    class Location;
    typedef std::vector<Location> Locations;
        
    class Location{
    protected:
        double x_ = 0;
        double y_ = 0;
        double z_ = 0;
        double floor_ = 0;
        
    public:
        using Ptr = std::shared_ptr<Location>;
        
        Location(){}
        Location(double x, double y, double z, double floor);
        ~Location(){}
        
        double x() const;
        double y() const;
        double floor() const;
        double z() const;
        
        Location& x(double x);
        Location& y(double y);
        Location& z(double z);
        Location& floor(double floor);
        
        std::string toString() const;
        
        void copyLocation(const Location& location);
        
        //static Location mean(const std::vector<Location>& locations);
        template <class Tlocation>
        static Location mean(const std::vector<Tlocation>& locations);
        template <class Tlocation>
        static Location weightedMean(const std::vector<Tlocation>& locations, const std::vector<double> weights);
        
        template <class Tlocation>
        static Location standardDeviation(const std::vector<Tlocation>& locations);
        
        static bool equals(const Location& location1, const Location& location2);
        static bool close(const Location& location1, const Location& location2, double epsilon);
        static double distance2D(const Location& location1, const Location& location2);
        static double distance(const Location& location1, const Location& location2);
        static double distance(const Location& loc1, const Location& loc2, double offset);
        static double floorDifference(const Location& location1, const Location& location2);
        static bool checkDifferentFloor(const Location& location1, const Location& location2);
        
        template <class Tlocation>
        static std::vector<Tlocation> filterLocationsOnFlatFloor(const std::vector<Tlocation>& locations);
        
        template <class Tlocation>
        static int findKNNDensestLocationIndex(const std::vector<Tlocation>& locs, int knn=-1, double floorCoeff=1000);
        template <class Tlocation, class Tstate>
        static int findClosestLocationIndex(const Tlocation& queryLocation, const std::vector<Tstate>& locs, double floorCoeff=1000);
        template <class Tlocation>
        static int findKDEDensestLocationIndex(const std::vector<Tlocation>& locs, double bandwidth=0.5, double floorCoeff=1000);
        
        // for container
        bool operator<(const Location &right) const;
        bool operator==(const Location &right) const;
        bool operator!=(const Location &right) const;
        
        // for string stream
        friend std::ostream& operator<<(std::ostream&os, const Location& location);
        
    };
    
        
    // LocationProperty
    class LocationProperty{
        double stdX_ = 0.0; //[m]
        double stdY_ = 0.0; //[m]
    public:
        LocationProperty& stdX(double stdX);
        double stdX() const;
        LocationProperty& stdY(double stdY);
        double stdY() const;
    };
    
    // Template functions
    template <class Tlocation>
    Location Location::mean(const std::vector<Tlocation>& locations){
        size_t n = locations.size();
        double w = 1.0/n;
        std::vector<double> weights(n, w);
        return weightedMean(locations, weights);
    }
    
    template <class Tlocation>
    Location Location::weightedMean(const std::vector<Tlocation>& locations, const std::vector<double> weights){
        
        double x = 0;
        double y = 0;
        double floor = 0;
        double z = 0;
        size_t n = locations.size();
        double weightSum = 0;
        for(int i=0; i<n; i++){
            weightSum += weights.at(i);
        }
        for(int i=0; i<n; i++){
            Tlocation loc = locations.at(i);
            double weight = weights.at(i)/weightSum;
            x += loc.x() * weight;
            y += loc.y() * weight;
            floor += loc.floor() * weight;
            z += loc.z() * weight;
        }
        
        Location meanLocation(x,y,z,floor);
        return meanLocation;
    }
    
    template <class Tlocation>
    Location Location::standardDeviation(const std::vector<Tlocation>& locations){
        Location meanLocation = mean(locations);
        double sumSqX = 0;
        double sumSqY = 0;
        double sumSqFloor = 0;
        double sumSqZ = 0;
        for(Location loc: locations){
            sumSqX += std::pow(loc.x() - meanLocation.x(), 2);
            sumSqY += std::pow(loc.y() - meanLocation.y(), 2);
            sumSqFloor += std::pow(loc.floor() - meanLocation.floor(), 2);
            sumSqZ += std::pow(loc.z() - meanLocation.z(), 2);
        }
        size_t n = locations.size();
        sumSqX/=n;
        sumSqY/=n;
        sumSqFloor/=n;
        sumSqZ/=n;
        Location stdevLocation(std::sqrt(sumSqX), std::sqrt(sumSqY), std::sqrt(sumSqZ), std::sqrt(sumSqFloor));
        return stdevLocation;
    }
    
    template <class Tlocation>
    std::vector<Tlocation> Location::filterLocationsOnFlatFloor(const std::vector<Tlocation>& locations){
        std::vector<Tlocation> locsNew;
        for(const auto& loc: locations){
            if(loc.z() == 0.0){
                locsNew.push_back(loc);
            }
        }
        return locsNew;
    }
}

#endif /* Location_hpp */
