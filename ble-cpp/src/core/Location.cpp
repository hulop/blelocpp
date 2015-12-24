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

#include "Location.hpp"
#include <sstream>
#include <cmath>

namespace loc{
    
    Location::Location(double x, double y, double z, double floor){
        x_ = x;
        y_ = y;
        z_ = z;
        floor_ = floor;
    }
    
    double Location::x() const{
        return x_;
    }
    
    double Location::y() const{
        return y_;
    }
    
    double Location::floor() const{
        return floor_;
    }
    
    double Location::z() const{
        return z_;
    }
    
    Location& Location::x(double x){
        x_ = x;
        return *this;
    }
    
    Location& Location::y(double y){
        y_ = y;
        return *this;
    }
    
    Location& Location::floor(double floor){
        floor_ = floor;
        return *this;
    }
    
    Location& Location::z(double z){
        z_ = z;
        return *this;
    }
    
    std::string Location::toString() const{
        std::stringstream stream;
        stream << x_ << ","<< y_ << "," << z_ << "," << floor_ ;
        std::string str = stream.str();
        return str;
    }
    
    bool Location::equals(const Location& loc1, const Location& loc2){
        if(   loc1.x() == loc2.x()
           && loc1.y()==loc2.y()
           && loc1.z() == loc2.z()
           && loc1.floor() == loc2.floor()){
            return true;
        }else{
            return false;
        }
    }
    
    bool Location::close(const Location& loc1, const Location& loc2, double epsilon){
        double dist = distance(loc1, loc2);
        if(dist <= epsilon){
            return true;
        }else{
            return false;
        }
    }
    
    double Location::distance2D(const Location& loc1, const Location& loc2){
        double dx = loc1.x() - loc2.x();
        double dy = loc1.y() - loc2.y();
        double dist = std::sqrt(dx*dx + dy*dy);
        return dist;
    }
    
    double Location::distance(const Location& loc1, const Location& loc2){
        double dx = loc1.x() - loc2.x();
        double dy = loc1.y() - loc2.y();
        double dz = loc1.z() - loc2.z();
        double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        return dist;
    }
    
    double Location::distance(const Location& loc1, const Location& loc2, double offset){
        double dx = loc1.x() - loc2.x();
        double dy = loc1.y() - loc2.y();
        double dz = loc1.z() - loc2.z();
        double dist = std::sqrt(dx*dx + dy*dy + dz*dz + offset*offset);
        return dist;
    }
    
    double Location::floorDifference(const Location& loc1, const Location& loc2){
        return std::abs(loc1.floor() - loc2.floor());
    }
    
    
    bool Location::operator<(const loc::Location &right) const{
        if(this->floor() < right.floor()){
            return true;
        }else if(this->floor() > right.floor()){
            return false;
        }
        
        if(this->z() < right.z()){
            return true;
        }else if(this->z() > right.z()){
            return false;
        }
        
        if(this->y() < right.y()){
            return true;
        }else if(this->y() > right.y()){
            return false;
        }
        
        if(this->x() < right.x()){
            return true;
        }else if(this->x() > right.x()){
            return false;
        }
        
        return false;
    }
    
    bool Location::operator==(const loc::Location &right) const{
        if(this->floor() == right.floor()
           && this->x() == right.x()
           && this->y() == right.y()
           && this->z() == right.z()){
            return true;
        }else{
            return false;
        }
    }
    
    bool Location::operator!=(const loc::Location &right) const{
        
        if(this->floor()!=right.floor()) return true;
        if(this->x()!=right.x()) return true;
        if(this->y()!=right.y()) return true;
        if(this->z()!=right.z()) return true;
        
        return false;
    }
    
    // for string stream
    std::ostream& operator<<(std::ostream&os, const Location& location){
        os << location.x() <<"," << location.y() <<"," <<location.z() <<"," <<location.floor();
        return os;
    }
    
}
