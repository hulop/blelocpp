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
#ifdef ANDROID_STL_EXT
#include "string_ext.hpp"
#endif /* ANDROID_STL_EXT */

namespace loc{
    class DataUtils{
     
    public:
        static std::list<std::string> splitCSV(const std::string & str);
        static std::list<std::string> splitAndTrimCSV(const std::string & str);
        template<class T> static std::vector<T> listToVector(const std::list<T>& listObject);
        
        static bool csvCheckSensorType(const std::string& str, const std::string& type);
        static bool csvCheckAcceleration(const std::string& str);
        static bool csvCheckAttitude(const std::string& str);
        static bool csvCheckBeacons(const std::string& str);
        
        template<class T> static T parseCSVSensorData(const std::string& str);
        static Acceleration parseAccelerationCSV(const std::string& str);
        static Attitude parseAttitudeCSV(const std::string& str);
        
        static Beacons parseBeaconsCSV(const std::string& str); //timestamp, Beacon, x, y, height, floor, nBeacon, major, minor, rssi, ...
        
        /**
         return a Beacons object by parsing a csv string
         
         @param str a csv string
                "Beacon",nBeacon,major,minor,rssi,....,timestamp
                or
                "Beacon",nBeacon,uuid-major-minor,rssi,....,timestamp
         @return a Beacons object
         */
        static Beacons parseLogBeaconsCSV(const std::string& str); // Beacon, nBeacon, major, minor, rssi, ...., timestamp
        
        static Beacon jsonBeaconObjectToBeacon(picojson::object& jsonObject);
        static Beacons jsonBeaconsObjectToBeacons(picojson::object& beaconsObj);
        static Location jsonInformationObjectToLocation(picojson::object& informationObject);
        static Samples jsonSamplesArrayToSamples(picojson::array& array);
        static Samples jsonSamplesStringToSamples(const std::string& str);
        
        static std::string fileToString(const std::string& filePath);
        static std::string stringToFile(const std::string& dataStr, const std::string& dir, const std::string& file = "");
        
        static Location parseLocationCSV(const std::string& csvLine);
        
        /**
         return a Sample object by parsing a csv string

         @param str a csv string
                    timestamp,"Beacon",x,y,z,floor,N,major1,minor1,rssi1,...,majorN,minorN,rssiN
                    or
                    timestamp,"Beacon",x,y,z,floor,N,uuid1-major1-minor1,rssi1,...,uuidN-majorN-minorN,rssiN         
         @return a Sample object
         */
        static Sample parseSampleCSV(const std::string& str)  throw (std::invalid_argument);
        static Sample parseSampleCSV(const std::string& str, bool noBeacons)  throw (std::invalid_argument);
        
        static void csvLocationsToLocations(std::istream& istream, Locations& locations);
        
        static void csvSamplesToSamples(std::istream& istream, Samples &Samples);
        static void csvSamplesToSamples(std::istream& istream, Samples &Samples, bool noBeacons);
        static Samples csvSamplesToSamples(std::istream& istream);
        static Samples csvSamplesToSamples(std::istream& istream, bool noBeacons);
        static Sample parseShortSampleCSV(const std::string& csvLine)  throw (std::invalid_argument);
        static void shortCsvSamplesToSamples(std::istream& istream, Samples &Samples);
        static Samples shortCsvSamplesToSamples(std::istream& istream);
        static std::string samplesToCsvSamples(const Samples& samples);
        
        static BLEBeacon parseBLEBeaconCSV(const std::string& csvLine) throw(...);
        static BLEBeacons csvBLEBeaconsToBLEBeacons(std::istream&);
        static std::string BLEBeaconsToCSV(const BLEBeacons& bleBeacons);
        
        static Pose parseResetPoseCSV(const std::string& str);
        
        template<class Tstate>
        static std::string statesToCSV(const std::vector<Tstate>& states);
        
        static picojson::object locationToJSONObject(Location location);
        static picojson::object poseToJSONObject(Pose pose);
        static picojson::object stateToJSONObject(State state);
        static picojson::array statesToJSONArray(std::vector<State> states);
        static picojson::array statesToJSONArrayLight(std::vector<State> states);
        static picojson::object statusToJSONObject(Status status, bool optOutputStates);
        
        static picojson::array beaconsToJSONArray(const Beacons& beacons);
    };
    
    // Implementation
    template<class Tstate>
    std::string DataUtils::statesToCSV(const std::vector<Tstate>& states){
        std::stringstream ss;
        for(const auto& s: states){
            ss << s << std::endl;
        }
        return ss.str();
    }
    
    template<class T> std::vector<T> DataUtils::listToVector(const std::list<T>& listObject){
        std::vector<T> vec;
        for(auto iter=listObject.begin(); iter!=listObject.end(); iter++){
            vec.push_back(*iter);
        }
        return vec;
    }
    
}

#endif /* DataUtils_hpp */
