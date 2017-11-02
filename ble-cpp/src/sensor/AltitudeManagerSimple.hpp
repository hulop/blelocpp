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

#ifndef AltitudeManagerSimple_hpp
#define AltitudeManagerSimple_hpp

#include <iostream>
#include <memory>
#include <vector>
#include <queue>
#include <deque>
#include <mutex>
#include "AltitudeManager.hpp"

namespace loc{
    
    class AltitudeManagerSimple: public AltitudeManager{
    public:
        using Ptr = std::shared_ptr<AltitudeManagerSimple>;
        
        class Parameters{
        public:
            using Ptr = std::shared_ptr<Parameters>;
            
            long timestampIntervalLimit_ = 3000;
            int queueLimit_ = 60;
            int window_ = 3;
            double stdThreshold_ = 0.15;
            
            int queueLimit() const{
                return queueLimit_;
            }
            long timestampIntervalLimit() const{
                return timestampIntervalLimit_;
            }
            int window() const{
                return window_;
            }
            double stdThreshold() const{
                return stdThreshold_;
            }
            
            void timestampIntervalLimit(long timestampInterval){
                timestampIntervalLimit_ = timestampInterval;
            }
            void queueLimit(int queueLim){
                queueLimit_ = queueLim;
            }
            void window(int win){
                window_ = win;
            }
            void stdThreshold(double stdThresh){
                stdThreshold_ = stdThresh;
            }
        };
        
        AltitudeManagerSimple() = default;
        ~AltitudeManagerSimple() = default;
        
        virtual void putAltimeter(Altimeter alt);
        virtual double heightChange() const;
        void parameters(std::shared_ptr<Parameters> params);
        void verbose(bool);
        bool verbose() const;
    
    protected:
        Parameters::Ptr mParams = std::shared_ptr<Parameters>(new Parameters);
        std::deque<Altimeter> altimeterQueue;
        bool verbose_ = false;
    private:
        std::mutex mtx_;
    };
}

#endif /* AltitudeManagerSimple_hpp */
