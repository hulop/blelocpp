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
#include "Pose.hpp"
#include "State.hpp"
#include <sstream>
#include <cmath>

#include <opencv2/core/core.hpp>
#include <opencv2/flann/flann.hpp>

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
        stream << x() << ","<< y() << "," << z() << "," << floor();
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
    
    bool Location::checkDifferentFloor(const Location& loc1, const Location& loc2){
        double fdiff = Location::floorDifference(loc1, loc2);
        return 1.0 <= fdiff;
    }
    
    void Location::copyLocation(const Location& location){
        x_ = location.x();
        y_ = location.y();
        z_ = location.z();
        floor_ = location.floor();
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
        os << location.toString();
        return os;
    }
    
    // LocationProperty
    LocationProperty& LocationProperty::stdX(double stdX){
        this->stdX_ = stdX;
        return *this;
    }
    
    double LocationProperty::stdX() const{
        return stdX_;
    }
    
    LocationProperty& LocationProperty::stdY(double stdY){
        this->stdY_ = stdY;
        return *this;
    }
    
    double LocationProperty::stdY() const{
        return stdY_;
    }
    
    
    template <class Tlocation>
    int Location::findKNNDensestLocationIndex(const std::vector<Tlocation>& locs, int knn, double floorCoeff){
        int n = (int) locs.size();
        if(knn<=0){
            knn = (int) n/10;
        }
        cv::Mat data = cv::Mat::zeros(n, 4, CV_32FC1);
        for(int i=0; i<n; i++){
            data.at<float>(i,0) = locs.at(i).x();
            data.at<float>(i,1) = locs.at(i).y();
            data.at<float>(i,2) = locs.at(i).z();
            data.at<float>(i,3) = floorCoeff*locs.at(i).floor();
        }
        cv::flann::Index idx(data, cv::flann::KDTreeIndexParams(), cvflann::FLANN_DIST_EUCLIDEAN);
        
        std::vector<double> knn_dists;
        for(int i=0; i<n; i++){
            auto query = data.row(i);
            std::vector<int> indices;
            std::vector<float> dists;
            idx.knnSearch(query, indices, dists, knn);
            double dist =  dists.back();
            knn_dists.push_back(dist);
        }
        
        auto result = std::min_element(knn_dists.begin(), knn_dists.end());
        auto index = std::distance(knn_dists.begin(), result);
        return index;
    }
    
    template <class Tlocation, class Tstate>
    int Location::findClosestLocationIndex(const Tlocation& queryLocation, const std::vector<Tstate>& locs, double floorCoeff){
        int n = (int) locs.size();
        cv::Mat data = cv::Mat::zeros(n, 4, CV_32FC1);
        for(int i=0; i<n; i++){
            data.at<float>(i,0) = locs.at(i).x();
            data.at<float>(i,1) = locs.at(i).y();
            data.at<float>(i,2) = locs.at(i).z();
            data.at<float>(i,3) = floorCoeff*locs.at(i).floor();
        }
        cv::flann::Index idx(data, cv::flann::KDTreeIndexParams(), cvflann::FLANN_DIST_EUCLIDEAN);
        
        std::vector<double> knn_dists;
        
        cv::Mat query = data.row(0).clone();
        query.at<float>(0) = queryLocation.x();
        query.at<float>(1) = queryLocation.y();
        query.at<float>(2) = queryLocation.z();
        query.at<float>(3) = floorCoeff * queryLocation.floor();

        std::vector<int> indices;
        std::vector<float> dists;
        idx.knnSearch(query, indices, dists, 1);
        int index = indices.at(0);
        return index;
    }
    
    template <class Tlocation>
    int Location::findKDEDensestLocationIndex(const std::vector<Tlocation>& locs, double bandwidth, double floorCoeff){
        int n = (int) locs.size();
        std::vector<double> knn_dists;
        for(int i=0; i<n; i++){
            auto loc = locs.at(i);
            double dist = 0;
            for(int j=0; j<n; j++){
                double d = Location::distance(loc, locs.at(j)) + floorCoeff*Location::floorDifference(loc, locs.at(j));
                dist += std::exp(-d*d/(bandwidth*bandwidth));
            }
            knn_dists.push_back(dist);
        }
        
        auto result = std::max_element(knn_dists.begin(), knn_dists.end());
        auto index = std::distance(knn_dists.begin(), result);
        return index;
    }
    
    template int Location::findKNNDensestLocationIndex<Location>(const std::vector<Location>& locs, int knn, double floorCoeff);
    template int Location::findKNNDensestLocationIndex<State>(const std::vector<State>& locs, int knn, double floorCoeff);
    template int Location::findKNNDensestLocationIndex<Pose>(const std::vector<Pose>& locs, int knn, double floorCoeff);
    
    template int Location::findClosestLocationIndex<Location, State>(const Location& queryLocation, const std::vector<State>& locs, double floorCoeff);
    template int Location::findClosestLocationIndex<Pose, State>(const Pose& queryLocation, const std::vector<State>& locs, double floorCoeff);
    template int Location::findClosestLocationIndex<State, State>(const State& queryLocation, const std::vector<State>& locs, double floorCoeff);
    
    template int Location::findKDEDensestLocationIndex<State>(const std::vector<State>& locs, double bandwidth, double floorCoeff);
    
    
    template <class Tlocation>
    double Location::compute2DVariance(const std::vector<Tlocation>& locations){
        auto meanLoc = Location::mean(locations);
        double varx = 0, vary = 0, covxy = 0; // = yx
        size_t n = locations.size();
        for(int i=0; i<n; i++){
            const Tlocation& loc = locations.at(i);
            double dx = loc.x() - meanLoc.x();
            double dy = loc.y() - meanLoc.y();
            varx += dx*dx;
            vary += dy*dy;
            covxy += dx*dy;
        }
        varx/=n;
        vary/=n;
        covxy/=n;
        double det = varx*vary - covxy*covxy;
        return det;
    }
    template double Location::compute2DVariance<Location>(const std::vector<Location>& locations);
    template double Location::compute2DVariance<Pose>(const std::vector<Pose>& locations);
    template double Location::compute2DVariance<State>(const std::vector<State>& locations);
    
    
}
