/*******************************************************************************
 * Copyright (c) 2018  IBM Corporation, Carnegie Mellon University and others
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

#ifndef CnnFileUtil_hpp
#define CnnFileUtil_hpp

#include <stdio.h>
#include <list>
#include <map>
#include <opencv2/core/core.hpp>
#include "CnnManager.hpp"

namespace loc{
    
    class CnnFileUtil{
    public:
        static std::list<std::string> splitSSV(const std::string & str);
        static std::list<std::string> splitAndTrimSSV(const std::string & str);
        
        static std::map<std::pair<int,int>,int> parseBeaconSettingFile(const std::string& beaconFilePath);
        static void parseBeaconString(const std::string &line, std::map<std::pair<int,int>,int> &beaconIndex, cv::Mat &qRssi);
        static void parseCnnSettingJsonFile(const std::string& cnnSettingFilePath,
                                            const std::string& cnnModelParentDirPath,
                                            loc::ImageLocalizeMode imageLocalizeMode,
                                            std::map<int, std::string>& cnnModelFileDict,
                                            std::map<int, std::string>& beaconSettingFileDict);
    };
    
}

#endif /* CnnFileUtil_hpp */
