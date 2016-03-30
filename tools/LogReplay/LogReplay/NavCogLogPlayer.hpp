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

#ifndef NavCogLogPlayer_hpp
#define NavCogLogPlayer_hpp

//#define NO_BOOST_DATE_TIME_INLINE
//#define BOOST_DATE_TIME_NO_LIB

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <functional>
#include <boost/algorithm/string.hpp>
#include <time.h>
//#include <boost/date_time/posix_time/posix_time.hpp>
//#include <boost/date_time/c_local_time_adjustor.hpp>

#include "DataUtils.hpp"

namespace loc{
    
    class NavCogLogPlayer{
    private:
        std::string mFilePath;
        bool mOneDPDR;
        float mStartY, mEndY;
        float orientation;
        
        std::function<void (Beacons)> mFuncBeacons = [](Beacons beacons){
            std::cout << beacons.timestamp() << ",Beacon,";
            for(auto b: beacons){
                std::cout<<","<< b;
            }
            std::cout << std::endl;
            };
        std::function<void (Acceleration)> mFuncAcc = [](Acceleration acc){
            std::cout << acc.timestamp()  << ",Acc," << acc << std::endl;
        };
        std::function<void (Attitude)> mFuncAtt = [](Attitude att){
            std::cout << att.timestamp() << ",Att," << att << std::endl;
        };
        std::function<void (Pose)> mFuncReset = [](Pose poseReset){
            std::cout << "Reset=" << poseReset << std::endl;
        };
        std::function<void (long, double)> mFuncReached = [](long time_stamp, double pos){
            std::cout << "Reached=" << pos << std::endl;
        };
            
            
        public:
            void oneDPDR(bool oneDPDR, float starty, float endy) {
                mOneDPDR = oneDPDR;
                mStartY = starty;
                mEndY = endy;
                if (mStartY < mEndY) {
                    orientation = M_PI_2;
                } else {
                    orientation = -M_PI_2;
                }
            }
            
            void filePath(const std::string& filePath){
                mFilePath = filePath;
            }
            
            void functionCalledWhenBeaconsUpdated(std::function<void(Beacons)> func){
                mFuncBeacons = func;
            }
            
            void functionCalledWhenAccelerationUpdated(std::function<void(Acceleration)> func){
                mFuncAcc = func;
            }
            
            void functionCalledWhenAttitudeUpdated(std::function<void(Attitude)> func){
                mFuncAtt = func;
            }
            
            void functionCalledWhenReset(std::function<void(Pose)> func){
                mFuncReset = func;
            }
        
            void functionCalledWhenReached(std::function<void(long, double)> func){
                mFuncReached = func;
            }
            
            void close(){
                
            }
            
            void run();
            
            void processLine(std::string strBuffer);
    };
            
}
            
#endif /* NavCogLogPlayer_hpp */
