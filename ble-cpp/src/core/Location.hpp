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
        
        //static Location mean(const std::vector<Location>& locations);
        template <class Tlocation>
        static Location mean(const std::vector<Tlocation>& locations);
        template <class Tlocation>
        static Location standardDeviation(const std::vector<Tlocation>& locations);
        
        static bool equals(const Location& location1, const Location& location2);
        static bool close(const Location& location1, const Location& location2, double epsilon);
        static double distance2D(const Location& location1, const Location& location2);
        static double distance(const Location& location1, const Location& location2);
        static double distance(const Location& loc1, const Location& loc2, double offset);
        static double floorDifference(const Location& location1, const Location& location2);
        
        // for container
        bool operator<(const Location &right) const;
        bool operator==(const Location &right) const;
        bool operator!=(const Location &right) const;
        
        // for string stream
        friend std::ostream& operator<<(std::ostream&os, const Location& location);
        
    };
    
    // Template functions
    
    template <class Tlocation>
    Location Location::mean(const std::vector<Tlocation>& locations){
        double x = 0;
        double y = 0;
        double floor = 0;
        double z = 0;
        size_t n = locations.size();
        for(Location loc: locations){
            x += loc.x();
            y += loc.y();
            floor += loc.floor();
            z += loc.z();
        }
        x/=n;
        y/=n;
        floor/=n;
        z/=n;
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
    
}

#endif /* Location_hpp */
