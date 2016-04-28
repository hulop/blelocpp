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

#include <iostream>
#include <string>
#include <algorithm>

#include "Location.hpp"
#include "Beacon.hpp"
#include "BLEBeacon.hpp"
#include "Attitude.hpp"
#include "Acceleration.hpp"

#include "DataUtils.hpp"

namespace loc{
    
    std::list<std::string> DataUtils::splitCSV(const std::string & str){
        std::list<std::string> stringList;
        std::string delim (",");
        boost::split(stringList, str, boost::is_any_of(delim));
        return stringList;
    }
    
    std::list<std::string> DataUtils::splitAndTrimCSV(const std::string & str){
        std::list<std::string> stringList = splitCSV(str);
        std::list<std::string>::iterator iter;
        for( iter= stringList.begin(); iter!=stringList.end(); iter++){
            boost::trim(*iter);
        }
        return stringList;
    }
    
    bool DataUtils::csvCheckSensorType(const std::string& str, const std::string& type){
        std::list<std::string> stringList = splitAndTrimCSV(str);
        std::list<std::string>::iterator iter;
        int i=0;
        for( iter= stringList.begin(); iter!=stringList.end(); iter++){
            //std::cout << *iter << std::endl;
            if(i==1){
                if(iter->compare(type)==0){
                    return true;
                }
            }
            i++;
        }
        return false;
    }
    
    bool DataUtils::csvCheckAcceleration(const std::string& str){
        return csvCheckSensorType(str, "Acc");
    }
    
    bool DataUtils::csvCheckAttitude(const std::string& str){
        return csvCheckSensorType(str, "Motion");
    }
    
    bool DataUtils::csvCheckBeacons(const std::string& str){
        return csvCheckSensorType(str, "Beacon");
    }
    
    template<class T> T DataUtils::parseCSVSensorData(const std::string& str){
        std::list<std::string> stringList = splitAndTrimCSV(str);
        std::list<std::string>::iterator iter;
        int i=0;
        long timestamp=0L;
        double x=0 , y=0, z=0;
        
        for( iter= stringList.begin(); iter!=stringList.end(); iter++){
            //std::cout << *iter << std::endl;
            if(i==0) timestamp = std::stol(*iter);
            if(i==2) x = std::stod(*iter);
            if(i==3) y = std::stod(*iter);
            if(i==4) z = std::stod(*iter);
            i++;
        }
        
        T data(timestamp, x, y, z);
        return data;
    }
    
    template Acceleration DataUtils::parseCSVSensorData<Acceleration>(const std::string& str);
    template Attitude DataUtils::parseCSVSensorData<Attitude>(const std::string& str);
    
    
    Acceleration DataUtils::parseAccelerationCSV(const std::string& str){
        return parseCSVSensorData<Acceleration>(str);
    }
    
    Attitude DataUtils::parseAttitudeCSV(const std::string& str){
        return parseCSVSensorData<Attitude>(str);
    }
    
    Beacons DataUtils::parseBeaconsCSV(const std::string& str){
        // timestamp, Beacon, x, y, height, floor, nBeacon, major, minor, rssi
        Beacons beacons;
        
        std::list<std::string> stringList = splitAndTrimCSV(str);
        std::list<std::string>::iterator iter;
        int i=0;
        
        long timestamp = 0;
        int nBeacons = 0;
        int major=0;
        int minor=0;
        double rssi = -100;
        
        for( iter= stringList.begin(); iter!=stringList.end(); iter++){
            if(i==0) timestamp = std::stol(*iter);
            beacons.timestamp(timestamp);
            if(i==6) nBeacons = std::stoi(*iter);
            if(i>6){
                if(i%3==1) major = std::stoi(*iter);
                if(i%3==2) minor = std::stoi(*iter);
                if(i%3==0){
                    rssi = std::stod(*iter);
                    Beacon b(major, minor, rssi);
                    beacons.push_back(b);
                }
            }
            i++;
        }
        return beacons;
    }
    
    
    Beacon DataUtils::jsonBeaconObjectToBeacon(picojson::object& jsonObject){
        double majorDouble = jsonObject["major"].get<double>();
        double minorDouble = jsonObject["minor"].get<double>();
        double rssi = jsonObject["rssi"].get<double>();
        if (rssi == 0) { rssi = -100; }
        Beacon beacon(majorDouble, minorDouble, rssi );
        return beacon;
    }
    
    
    Beacons DataUtils::jsonBeaconsObjectToBeacons(picojson::object& beaconsObj){
        Beacons beacons;
        
        std::string uuid = beaconsObj["uuid"].get<std::string>();
        double timestampDouble = beaconsObj["timestamp"].get<double>();
        long timestamp = timestampDouble;
        beacons.timestamp(timestamp);
        
        picojson::array dataArray = beaconsObj["data"].get<picojson::array>();
        for(picojson::array::iterator iter_data=dataArray.begin() ; iter_data!=dataArray.end() ; iter_data++){
            picojson::object beaconObj = iter_data->get<picojson::object>();
            Beacon b = jsonBeaconObjectToBeacon(beaconObj);
            beacons.push_back(b);
        }
        return beacons;
    }
    
    
    Location DataUtils::jsonInformationObjectToLocation(picojson::object& informationObject){
        double x = informationObject["absx"].get<double>();
        double y = informationObject["absy"].get<double>();
        double z = informationObject["z"].get<double>();
        double floor = informationObject["floor_num"].get<double>();
        Location location(x,y,z,floor);
        return location;
    }
    
    
    Samples DataUtils::jsonSamplesArrayToSamples(picojson::array& array){
        Samples samples;
        for(picojson::array::iterator iter=array.begin(); iter!=array.end(); iter++){
            picojson::object obj = iter->get<picojson::object>();
            picojson::object info = obj["information"].get<picojson::object>();
            
            loc::Location location = jsonInformationObjectToLocation(info);
            
            picojson::array beaconsArray = obj["beacons"].get<picojson::array>();
            for(picojson::array::iterator itb = beaconsArray.begin(); itb!=beaconsArray.end(); itb++){
                picojson::object beaconsObj = itb->get<picojson::object>();
                loc::Beacons beacons = jsonBeaconsObjectToBeacons(beaconsObj);
                
                Sample sample;
                sample.timestamp(beacons.timestamp())->beacons(beacons)->location(location);
                
                samples.push_back(sample);
            }
        }
        return samples;
    }
    
    Samples DataUtils::jsonSamplesStringToSamples(const std::string& str){
        picojson::value value;
        std::string err = picojson::parse(value, str);
        picojson::array& array = value.get<picojson::array>();
        return jsonSamplesArrayToSamples(array);
    }
    
    std::string DataUtils::fileToString(const std::string& filePath){
        std::ifstream ifs(filePath);
        
        if(! ifs.is_open()){
            std::cout << filePath << " is not open." << std::endl;
        }
        std::string str;
        std::stringstream stringstream;
        std::string strBuffer;
        while(std::getline(ifs, strBuffer)){
            stringstream << strBuffer;
        }
        return stringstream.str();
    }
    
    Location DataUtils::parseLocationCSV(const std::string& csvLine){
        
        std::list<std::string> stringList = splitAndTrimCSV(csvLine);
        std::list<std::string>::iterator iter;
        
        // timestamp, type, x, y, height ,floor(z), #beacons, major, minor, rssi ,...
        std::vector<std::string> stringVector;
        for( iter= stringList.begin(); iter!=stringList.end(); iter++){
            stringVector.push_back(*iter);
        }
        double x = std::stod(stringVector.at(2));
        double y = std::stod(stringVector.at(3));
        double z = std::stod(stringVector.at(4));
        double floor = std::stod(stringVector.at(5));
        
        Location location(x,y,z,floor);
        return location;
    }
    
    Sample DataUtils::parseSampleCSV(const std::string& csvLine) throw(std::invalid_argument) {
        //Location location = parseLocationCSV(csvLine);
        //Beacons beacons = parseBeaconsCSV(csvLine);
        
        const char *buffer = csvLine.c_str();
        long timestamp;
        double x, y, z, floor;
        int num, n = 0;
        sscanf(buffer, "%ld,Beacon,%lf,%lf,%lf,%lf,%d,%n", &timestamp, &x, &y, &z, &floor, &num, &n);
        
        if(n==0){
            throw std::invalid_argument("Invalid csv line was found.");
        }
        
        buffer += n;
        Location location(x,y,z,floor);
        
        int major, minor;
        double rssi;
        Beacons beacons;
        beacons.timestamp(timestamp);
        for(int i = 0; i < num; i++) {
            sscanf(buffer, "%d,%d,%lf,%n", &major, &minor, &rssi, &n);
            buffer += n;
            beacons.push_back(Beacon(major, minor, rssi));
        }
        
        Sample sample;
        sample.timestamp(timestamp)->location(location)->beacons(beacons);
        return sample;
    }
    
    void DataUtils::csvSamplesToSamples(std::istream& istream, Samples &samples){
        std::string strBuffer;
        while(std::getline(istream, strBuffer)){
            try{
                Sample sample = parseSampleCSV(strBuffer);
                samples.push_back(sample);
            } catch (std::invalid_argument e){
                std::cout << "Invalid csv line was found. line=" <<strBuffer << std::endl;
                //std::cout << "Header line is found in csv samples." << std::endl;
            }
        }
    }
    
    Samples DataUtils::csvSamplesToSamples(std::istream& istream){
        Samples samples;
        std::string strBuffer;
        while(std::getline(istream, strBuffer)){
            try{
                Sample sample = parseSampleCSV(strBuffer);
                samples.push_back(sample);
            } catch (std::invalid_argument e){
                std::cout << "Invalid csv line was found. line=" <<strBuffer << std::endl;
            }
        }
        if(samples.size()==0){
            std::cout << "Samples CSV was not found." << std::endl;
        }
        return std::move(samples);
    }
    
    Sample DataUtils::parseShortSampleCSV(const std::string& csvLine) throw(std::invalid_argument) {
        //Location location = parseLocationCSV(csvLine);
        //Beacons beacons = parseBeaconsCSV(csvLine);
        
        const char *buffer = csvLine.c_str();
        long timestamp = 0;
        double x, y, z = 0, floor = 0;
        int num, n = 0;
        sscanf(buffer, "%lf,%lf,%d,%n", &x, &y, &num, &n);
        
        if(n==0){
            throw std::invalid_argument("Invalid csv line was found.");
        }
        
        buffer += n;
        Location location(x,y,z,floor);
        
        int major, minor;
        double rssi;
        Beacons beacons;
        beacons.timestamp(timestamp);
        for(int i = 0; i < num; i++) {
            sscanf(buffer, "%d,%d,%lf,%n", &major, &minor, &rssi, &n);
            buffer += n;
            beacons.push_back(Beacon(major, minor, rssi));
        }
        
        Sample sample;
        sample.timestamp(timestamp)->location(location)->beacons(beacons);
        return sample;
    }
    
    void DataUtils::shortCsvSamplesToSamples(std::istream& istream, Samples &samples){
        std::string strBuffer;
        while(std::getline(istream, strBuffer)){
            try{
                Sample sample = parseShortSampleCSV(strBuffer);
                samples.push_back(sample);
            } catch (std::invalid_argument e){
                std::cout << "Invalid short csv line was found. line=" <<strBuffer << std::endl;
                //std::cout << "Header line is found in csv samples." << std::endl;
            }
        }
    }

    Samples DataUtils::shortCsvSamplesToSamples(std::istream& istream){
        Samples samples;
        std::string strBuffer;
        while(std::getline(istream, strBuffer)){
            try{
                Sample sample = parseShortSampleCSV(strBuffer);
                samples.push_back(sample);
            } catch (std::invalid_argument e){
                std::cout << "Invalid short csv line was found. line=" <<strBuffer << std::endl;
            }
        }
        if(samples.size()==0){
            std::cout << "Samples CSV was not found." << std::endl;
        }
        return std::move(samples);
    }
    
    
    
    BLEBeacon DataUtils::parseBLEBeaconCSV(const std::string& csvLine) throw (std::invalid_argument){
        std::list<std::string> stringList = splitAndTrimCSV(csvLine);
        std::list<std::string>::iterator iter;
        // uuid, major, minor, x, y, z, floor
        std::vector<std::string> stringVector;
        for( iter= stringList.begin(); iter!=stringList.end(); iter++){
            stringVector.push_back(*iter);
        }
        
        std::string uuid = stringVector.at(0);
        int major = std::stoi(stringVector.at(1));
        int minor = std::stoi(stringVector.at(2));
        
        double x = std::stod(stringVector.at(3));
        double y = std::stod(stringVector.at(4));
        double z = std::stod(stringVector.at(5));
        double floor = std::stod(stringVector.at(6));
        
        BLEBeacon bleBeacon(uuid, major, minor, x, y, z, floor);
        return bleBeacon;
        
    }
    
    BLEBeacons DataUtils::csvBLEBeaconsToBLEBeacons(std::istream& istream){
        BLEBeacons bleBeacons;
        std::string strBuffer;
        while(std::getline(istream, strBuffer)){
            try{
                BLEBeacon bleBeacon = parseBLEBeaconCSV(strBuffer);
                bleBeacons.push_back(bleBeacon);
            }catch(std::invalid_argument e){
                std::cout << "Invalid csv line was found. line=" <<strBuffer << std::endl;
                //std::cout << "Header line is found in csv BLEBeacons." << std::endl;
            }
        }
        
        if(bleBeacons.size()==0){
            std::cout << "BLEBeacons CSV was not found." << std::endl;
        }
        
        return bleBeacons;
    }
    
    
    
    Pose DataUtils::parseResetPoseCSV(const std::string& csvLine){
        
        std::list<std::string> stringList = splitAndTrimCSV(csvLine);
        std::list<std::string>::iterator iter;
        
        // timestamp, "Reset", x, y, floor, orientation
        std::vector<std::string> stringVector;
        for( iter= stringList.begin(); iter!=stringList.end(); iter++){
            stringVector.push_back(*iter);
        }
        double x = std::stod(stringVector.at(2));
        double y = std::stod(stringVector.at(3));
        double z = 0.0;
        double floor = std::stod(stringVector.at(4));
        double orientation = std::stod(stringVector.at(5));
        
        Pose pose;
        pose.x(x).y(y).z(z).floor(floor).orientation(orientation);
        pose.velocity(0);
        pose.normalVelocity(0.0);
        return pose;
    }
    
    picojson::object DataUtils::locationToJSONObject(Location location){
        picojson::object obj;
        obj.insert(std::make_pair("x", picojson::value(location.x())));
        obj.insert(std::make_pair("y", picojson::value(location.y())));
        obj.insert(std::make_pair("z", picojson::value(location.z())));
        obj.insert(std::make_pair("floor", picojson::value(location.floor())));
        return obj;
    }
    
    picojson::object DataUtils::poseToJSONObject(Pose pose){
        picojson::object obj = locationToJSONObject(pose);
        obj.insert(std::make_pair("orientation", picojson::value(pose.orientation())));
        obj.insert(std::make_pair("velocity", picojson::value(pose.velocity())));
        obj.insert(std::make_pair("normalVelocity", picojson::value(pose.normalVelocity())));
        return obj;
    }
    
    picojson::object DataUtils::stateToJSONObject(State state){
        picojson::object obj = poseToJSONObject(state);
        obj.insert(std::make_pair("orientationBias", picojson::value(state.orientationBias())));
        obj.insert(std::make_pair("rssiBias", picojson::value(state.rssiBias())));
        obj.insert(std::make_pair("weight", picojson::value(state.weight())));
        return obj;
    }
    
    
    picojson::array DataUtils::statesToJSONArray(std::vector<State> states){
        picojson::array array;
        for(State s: states){
            picojson::object obj = DataUtils::stateToJSONObject(s);
            array.push_back(picojson::value(obj));
        }
        return array;
    }
    
    picojson::array DataUtils::statesToJSONArrayLight(std::vector<State> states){
        picojson::array array;
        for(State s: states){
            picojson::array ar;
            ar.push_back(picojson::value(s.x()));
            ar.push_back(picojson::value(s.y()));
            ar.push_back(picojson::value(s.floor()));
            array.push_back(picojson::value(ar));
        }
        return array;
    }
    
    
    picojson::object DataUtils::statusToJSONObject(Status status, bool optOutputStates){
        std::shared_ptr<Location> meanLocation = status.meanLocation();
        std::shared_ptr<Pose> meanPose = status.meanPose();
        std::shared_ptr<std::vector<State>> states = status.states();
        Location stdevLocation = Location::standardDeviation(*states);
        
        picojson::object json;
        if(meanPose){
            json = poseToJSONObject(*meanPose);
        }else{
            json = locationToJSONObject(*meanLocation);
        }
        json.insert(std::make_pair("stdev", picojson::value(locationToJSONObject(stdevLocation))));
        if(optOutputStates){
            picojson::array array = statesToJSONArrayLight(*states);
            json.insert(std::make_pair("states", picojson::value(array)));
        }
        return json;
    }
    
    picojson::array DataUtils::beaconsToJSONArray(const Beacons& beacons){
        picojson::array array;
        for(const Beacon& b: beacons){
            picojson::object obj;
            obj.insert(std::make_pair("major", picojson::value((double)b.major())));
            obj.insert(std::make_pair("minor", picojson::value((double)b.minor())));
            obj.insert(std::make_pair("rssi", picojson::value(b.rssi())));
            array.emplace_back(obj);
        }
        return array;
    }
    
}
