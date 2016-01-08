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

#ifndef VirtualDevice_hpp
#define VirtualDevice_hpp

#include <stdio.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <stdexcept>
#include <fstream>
#include <time.h>
#include "bleloc.h"
#include "StreamLocalizer.hpp"
#include "DataUtils.hpp"
#include "DataLogger.hpp"

namespace loc{
    
    class VirtualDevice{
        
    private:
        int count_putBeacons = 0;
        long timestampMargin = 1000; //[ms]
        long prevTimestamp = 0;
        bool wasReset = false;
        
        std::string mCsvPath = "";
        std::shared_ptr<StreamLocalizer> mStreamLocalizer;
        
        std::string mResultDir = "";
        bool savesStates = true;
        
        std::stringstream sstream;
        
        Pose stdevPose_;
        double mCurrentError = 0;
        
    public:
        VirtualDevice() = default;
        
        VirtualDevice(Pose stdevPose): stdevPose_(stdevPose){}
        ~VirtualDevice() = default;
        
        double currentError() const;
        void setStdevPose(Pose stdevPose);
        void csvPath(std::string csvPath);
        void resultDir(std::string resultDir);
        void streamLocalizer(std::shared_ptr<StreamLocalizer> streamLocalizer);
        std::shared_ptr<StreamLocalizer> streamLocalizer() const;
        void resetLocalizer();
        void checkTimestampConsistency(std::string str);
        void checkTimestampConsistency(long timestamp);
        
        // Interface method called by JNI
        Status update(Status status, std::string strBuffer);
        
        void processLine(std::string strBuffer);
        void run();
        void close();
    };
}

#endif /* VirtualDevice_hpp */
