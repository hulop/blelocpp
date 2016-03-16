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

#ifndef DataUtils_hpp
#define DataUtils_hpp

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <boost/algorithm/string.hpp>

#include "bleloc.h"
#include "BLEBeacon.hpp"
#include "picojson.h"

namespace loc{
    class DataUtils{
     
    public:
        static std::list<std::string> splitCSV(const std::string & str);
        static std::list<std::string> splitAndTrimCSV(const std::string & str);
        
        static bool csvCheckSensorType(const std::string& str, const std::string& type);
        static bool csvCheckAcceleration(const std::string& str);
        static bool csvCheckAttitude(const std::string& str);
        static bool csvCheckBeacons(const std::string& str);
        
        template<class T> static T parseCSVSensorData(const std::string& str);
        static Acceleration parseAccelerationCSV(const std::string& str);
        static Attitude parseAttitudeCSV(const std::string& str);
        static Beacons parseBeaconsCSV(const std::string& str);
        
        static Beacon jsonBeaconObjectToBeacon(picojson::object& jsonObject);
        static Beacons jsonBeaconsObjectToBeacons(picojson::object& beaconsObj);
        static Location jsonInformationObjectToLocation(picojson::object& informationObject);
        static Samples jsonSamplesArrayToSamples(picojson::array& array);
        static Samples jsonSamplesStringToSamples(const std::string& str);
        
        static std::string fileToString(const std::string& filePath);
        
        static Location parseLocationCSV(const std::string& csvLine);
        static Sample parseSampleCSV(const std::string& csvLine)  throw (std::invalid_argument);
        static void csvSamplesToSamples(std::istream& istream, Samples &Samples);
        static Samples csvSamplesToSamples(std::istream& istream);
        static Sample parseShortSampleCSV(const std::string& csvLine)  throw (std::invalid_argument);
        static void shortCsvSamplesToSamples(std::istream& istream, Samples &Samples);
        static Samples shortCsvSamplesToSamples(std::istream& istream);
        
        static BLEBeacon parseBLEBeaconCSV(const std::string& csvLine) throw (std::invalid_argument);
        static BLEBeacons csvBLEBeaconsToBLEBeacons(std::istream&);
        
        static Pose parseResetPoseCSV(const std::string& str);
        
        static picojson::object locationToJSONObject(Location location);
        static picojson::object poseToJSONObject(Pose pose);
        static picojson::object stateToJSONObject(State state);
        static picojson::array statesToJSONArray(std::vector<State> states);
        static picojson::array statesToJSONArrayLight(std::vector<State> states);
        static picojson::object statusToJSONObject(Status status, bool optOutputStates);
        
    };
    
}

#endif /* DataUtils_hpp */
