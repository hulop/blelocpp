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

namespace loc{

    class OrientationDriftAdjuster{
    public:
        class Parameters{
        public:
            double maxDifference = 0.1;
            double averagingRate = 0.001;
            long queueLimit = 100;
            long timestampIntervalLimit = 1000;
        public:
            using Ptr = std::shared_ptr<Parameters>;
        };
        
        using Ptr = std::shared_ptr<OrientationDriftAdjuster>;
        
    private:
        std::mutex mtx_;
    protected:
        std::deque<Attitude> attitudeQueue;
        double driftCoeffaveraged = 0.0;
        double currentDrift = 0.0;
        bool verbose = false;
        
        void reset(){
            driftCoeffaveraged = 0;
            currentDrift = 0;
            attitudeQueue.clear();
        }
        
    public:
        Parameters::Ptr mParams = std::make_shared<Parameters>();
        
        Attitude adjustAttitude(const Attitude& att){
            std::lock_guard<std::mutex> lock(mtx_);
            if(attitudeQueue.size() < (mParams->queueLimit)){
                std::cout << "queue.size <= queueLimit: timestamp=" << att.timestamp() << std::endl;
                attitudeQueue.push_back(att);
                return att;
            }
            
            Attitude last = attitudeQueue.back();
            
            if(std::abs(last.timestamp() - att.timestamp()) > mParams->timestampIntervalLimit){
                if(verbose){
                    std::cout << "timestamp interval between two altimeters is too large. altitudeManager was reset." << std::endl;
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
            if(mParams->maxDifference <= std::abs(instantDrift)){
                instantDrift = 0.0;
            }
            
            auto rate = mParams->averagingRate;
            double dt_short = (att.timestamp() - last.timestamp()) * 0.001;
            if(instantDrift!=0.0){
                driftCoeffaveraged = (1.0-rate)*driftCoeffaveraged + rate * instantDrift;
            }
            
            currentDrift = currentDrift - driftCoeffaveraged*dt_short;
            double yawAdj = att.yaw() + currentDrift;
            Attitude attAdj(att);
            attAdj.yaw(yawAdj);
            
            if(verbose){
                std::cout << "YawAdjustement: timestamp=" << att.timestamp() <<", yaw=" << att.yaw() << ", yawAdj=" << yawAdj <<", currentDrift=" << currentDrift
                << ", dt_long=" << dt_long << ",dt_short=" << dt_short  << ",queue.size=" << attitudeQueue.size() << std::endl;
            }
                
            return attAdj;
        }
    };
    
}
#endif /* OrientationAdjuster_hpp */
