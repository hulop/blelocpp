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

#ifndef OrientationAdjuster_hpp
#define OrientationAdjuster_hpp

#include <iostream>
#include <memory>
#include <deque>
#include <mutex>

#include "Attitude.hpp"
#include "Heading.hpp"

#define degToRad(deg) (deg*M_PI/180.0)
#define radToDeg(rad) (rad*180.0/M_PI)

namespace loc{
    
    // start
    class OrientationDriftAdjusterSimple{
    public:
        
        class Parameters{
        private:
            // yaw
            double maxYawRate = degToRad(30.0);
            double averagingCoefficient = 0.001;
            long queueLimit = 1;
            long timestampIntervalLimit = 1000;
            
            // heading
            double maxHeadingRate = degToRad(30.0);
            double maxHeadingAccuracy = degToRad(180.0);
            double timestampHeadingExtra = 100.0; //[ms]
            
            long headingQueueLimit = 1000;
            bool usesHeading = true;
            bool usesHeadingValue = true;
            
            friend OrientationDriftAdjusterSimple;
        public:
            using Ptr = std::shared_ptr<Parameters>;
        };
        
        using Ptr = std::shared_ptr<OrientationDriftAdjusterSimple>;
        
    private:
        std::mutex mtx_;
        std::mutex mtxHead_;
    protected:
        std::deque<Attitude> attitudeQueue;
        double driftCoeffAveraged = 0.0;
        double currentDrift = 0.0;
        std::deque<Heading> headingQueue;
        
        void reset(){
            driftCoeffAveraged = 0;
            currentDrift = 0;
            attitudeQueue.clear();
            headingQueue.clear();
        }
        
    public:
        bool verbose = false;
        Parameters::Ptr mParams = std::make_shared<Parameters>();
        
        NormalParameter linearInterpolateHeading(long timestamp, const Heading& h1, const Heading& h2) const{
            
            if(h2.timestamp() <= h1.timestamp()){
                std::stringstream ss;
                ss << "h2.timestamp(" << h2.timestamp() << ") <= h1.timestamp(" << h1.timestamp() <<"): " << std::endl;
                throw std::runtime_error(ss.str());
            }
            assert(h1.timestamp() < h2.timestamp());
            
            double dt = (timestamp-h1.timestamp())*0.001;
            double dT = (h2.timestamp()-h1.timestamp())*0.001;
            
            double h1Rad = degToRad(h1.magneticHeading());
            double h2Rad = degToRad(h2.magneticHeading());
            double h1CC = std::atan2(std::cos(h1Rad), std::sin(h1Rad));
            double h2CC = std::atan2(std::cos(h2Rad), std::sin(h2Rad));
            
            double h1Acc = degToRad(h1.headingAccuracy());
            double h2Acc = degToRad(h2.headingAccuracy());
            
            double hAcc = -1;
            if(0<=h1Acc and 0<=h2Acc){
                hAcc = std::max(h1Acc, h2Acc);
            }
            
            double ht = h1CC + (h2CC-h1CC)*dt/dT;
            return NormalParameter(ht, hAcc);
        }
        
        NormalParameter interpolateHeading(long timestamp){
            std::lock_guard<std::mutex> lock(mtxHead_);
            if(headingQueue.size()<3){
                NormalParameter np(0,-1);
                return np;
            }
            auto& q = headingQueue;
            
            if(timestamp < q.front().timestamp()){
                if(q.front().timestamp() - timestamp > mParams->timestampHeadingExtra){
                    double headingRad = degToRad(q.front().magneticHeading());
                    double heading = std::atan2(std::cos(headingRad), std::sin(headingRad));
                    double headingAccuracy = -1;
                    return NormalParameter(heading, headingAccuracy);
                }else{
                    Heading h1 = q.at(0);
                    Heading h2 = q.at(1);
                    return linearInterpolateHeading(timestamp, h1, h2);
                }
            }else if (q.back().timestamp() < timestamp){
                if(timestamp - q.back().timestamp() > mParams->timestampHeadingExtra){
                    double headingRad = degToRad(q.back().magneticHeading());
                    double heading = std::atan2(std::cos(headingRad), std::sin(headingRad));
                    double headingAccuracy = -1;
                    return NormalParameter(heading, headingAccuracy);
                }else{
                    Heading h1 = q.at(q.size()-2);
                    Heading h2 = q.at(q.size()-1);
                    return linearInterpolateHeading(timestamp, h1, h2);
                }
            }else{
                for(int i=0; i<headingQueue.size(); i++){
                    size_t n = headingQueue.size();
                    Heading h1 = headingQueue.at(n-1-i-1);
                    Heading h2 = headingQueue.at(n-1-i);
                    if(h1.timestamp() <= timestamp
                       && timestamp <= h2.timestamp()){
                        return linearInterpolateHeading(timestamp, h1, h2);
                    }
                }
                double headingRad = degToRad(q.back().magneticHeading());
                double heading = std::atan2(std::cos(headingRad), std::sin(headingRad));
                double headingAccuracy = -1;
                return NormalParameter(heading, headingAccuracy);
            }
        }
        
        Attitude adjustAttitude(const Attitude& attitude){
            std::lock_guard<std::mutex> lock(mtx_);
            if(attitudeQueue.size() < (mParams->queueLimit)){
                if(verbose){
                    std::cout << "attitudeQueue.size <= queueLimit: timestamp=" << attitude.timestamp() << std::endl;
                }
                attitudeQueue.push_back(attitude);
                return attitude;
            }
            
            auto att = attitude;
            Attitude last = attitudeQueue.back();
            double rad1 = last.yaw();
            double rad2 = att.yaw();
            double rad3 = calculateIncrementalOrientation(rad1, rad2);
            att.yaw(rad3);
            
            if(std::abs(last.timestamp() - att.timestamp()) > mParams->timestampIntervalLimit){
                if(verbose){
                    std::cout << "timestamp interval between two attitude is too large. orientationAdjuster was reset." << std::endl;
                }
                this->reset();
                attitudeQueue.push_back(att);
                return att;
            }
            
            Attitude front = attitudeQueue.front();
            attitudeQueue.pop_front();
            attitudeQueue.push_back(att);
            
            double dt_long = static_cast<double>(att.timestamp() - front.timestamp()) * 0.001;
            double instantDrift = (att.yaw() - front.yaw())/dt_long;
            
            if(verbose){
                std::cout << "instantDrift," << instantDrift << std::endl;
            }
            if(std::abs(instantDrift) < mParams->maxYawRate){
                if(mParams->usesHeading){
                    auto hFront = interpolateHeading(front.timestamp());
                    auto hCurrent =  interpolateHeading(att.timestamp());
                    double h_front = hFront.mean();
                    double h_acc_front = hFront.stdev();
                    double h_current = hCurrent.mean();
                    double h_acc_current = hCurrent.stdev();
                    double vHeading = (h_current - h_front)/dt_long;
                    if(verbose){
                        std::cout << "vHeading=" << vHeading << ", hFront=" << h_front << ", hCurrent=" << h_current <<
                        ",hFrontACC=" << h_acc_front << ",hCurrenAcc=" << h_acc_current << std::endl;
                    }
                    if(std::abs(vHeading) < mParams->maxHeadingRate
                       && 0<=h_acc_front && h_acc_front<mParams->maxHeadingAccuracy
                       && 0<=h_acc_current && h_acc_current<mParams->maxHeadingAccuracy){
                        if(mParams->usesHeadingValue) {
                            instantDrift = instantDrift - vHeading;
                        }else{
                            instantDrift = instantDrift;
                        }
                    }else{
                        instantDrift = 0.0;
                    }
                }else{
                    instantDrift = instantDrift;
                }
            }else{
                instantDrift = 0.0;
            }
            
            auto rate = mParams->averagingCoefficient;
            double dt_short = (att.timestamp() - last.timestamp()) * 0.001;
            if(instantDrift!=0.0){
                driftCoeffAveraged = (1.0-rate)*driftCoeffAveraged + rate * instantDrift;
            }
            
            currentDrift = currentDrift + driftCoeffAveraged*dt_short;
            double yawAdj = att.yaw() - currentDrift;
            Attitude attAdj(att);
            attAdj.yaw(yawAdj);
            
            if(verbose){
                std::cout << "YawAdjustement: timestamp=" << att.timestamp() <<", yaw=" << att.yaw() << ", yawAdj=" << yawAdj <<", currentDrift=" << currentDrift
                << ", dt_long=" << dt_long << ",dt_short=" << dt_short  << ",queue.size=" << attitudeQueue.size() << std::endl;
            }
            
            return attAdj;
        }
        
        double getCurrentDrift() const{
            return currentDrift;
        }
        
        double getCurrentDriftRate() const{
            return driftCoeffAveraged;
        }
        
        double calculateIncrementalOrientation(double oPrev, double oTemp){
            double x1 = std::cos(oPrev);
            double y1 = std::sin(oPrev);
            double x2 = std::cos(oTemp);
            double y2 = std::sin(oTemp);
            
            double inc = std::asin(x1*y2 - y1*x2);
            return oPrev + inc;
        }
        
        void putHeading(const Heading& heading){
            std::lock_guard<std::mutex> lock(mtxHead_);
            if(headingQueue.size()==0){
                headingQueue.push_back(heading);
            }else{
                auto last = headingQueue.back();
                auto headTmp = heading;
                
                double rad1 = degToRad(last.magneticHeading());
                double rad2 = degToRad(headTmp.magneticHeading());
                double rad = calculateIncrementalOrientation(rad1, rad2);
                double deg = radToDeg(rad);
                headTmp.magneticHeading(deg);
                
                if(std::abs(last.timestamp() - heading.timestamp()) > mParams->timestampIntervalLimit){
                    this->reset();
                    if(verbose){
                        std::cout << "timestamp interval between two heading is too large. orientationAdjuster was reset." << std::endl;
                    }
                }
                if(mParams->headingQueueLimit < headingQueue.size()){
                    headingQueue.pop_front();
                }
                if(last.timestamp()<heading.timestamp()){
                    headingQueue.push_back(headTmp);
                }else if(last.timestamp()==heading.timestamp()){
                    headingQueue.pop_back();
                    headingQueue.push_back(headTmp);
                }
            }
        }
    };
}
#endif /* OrientationAdjuster_hpp */
