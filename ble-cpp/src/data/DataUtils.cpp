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
#include <regex>

#include "Location.hpp"
#include "Beacon.hpp"
#include "BLEBeacon.hpp"
#include "Attitude.hpp"
#include "Acceleration.hpp"

#include "DataUtils.hpp"
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/archive/iterators/binary_from_base64.hpp>
#include <boost/archive/iterators/transform_width.hpp>
#include <boost/algorithm/string/replace.hpp>

//http://stackoverflow.com/questions/10521581/base64-encode-using-boost-throw-exception
using namespace boost::archive::iterators;
typedef
transform_width< binary_from_base64<std::string::const_iterator>, 8, 6 > it_binary_t;
//

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
    
    std::string removeSpaces(const std::string& str){
        const std::string str2 = boost::algorithm::replace_all_copy(str, " ", "");
        return str2;
    }
    
    Beacons DataUtils::parseBeaconsCSV(const std::string& str){
        const std::string str2 = removeSpaces(str);
        Sample s = parseSampleCSV(str2);
        return s.beacons();
    }
    
    /*
    Beacons DataUtils::parseBeaconsCSV(const std::string& str){
        // timestamp, Beacon, x, y, height, floor, nBeacon, major, minor, rssi, ...
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
    */
    
    // param (str) "uuid-major-minor"
    // return BeaconId
    BeaconId parseBeaconIdString(const std::string& str){
        std::istringstream iss(str);
        std::string newCell;
        std::vector<std::string> tokens;
        while( std::getline( iss, newCell, '-' ) ) {
            tokens.push_back(newCell);
        }
        
        auto n = tokens.size();
        std::string uuid;
        for(int i=0; i<n-3; i++){
            uuid += tokens[i] + "-";
        }
        uuid += tokens[n-3];
        
        int major = stoi(tokens.at(n-2));
        int minor = stoi(tokens.at(n-1));
        return BeaconId(uuid, major, minor);
    }
    
    Beacons DataUtils::parseLogBeaconsCSV(const std::string& str){
        // "Beacon",nBeacon,major,minor,rssi,....,timestamp
        // or "Beacon",nBeacon,uuid-major-minor,rssi,....,timestamp
        Beacons beacons;
        
        std::list<std::string> stringList = splitAndTrimCSV(str);
        std::list<std::string>::iterator iter;
        int i=0;
        
        long timestamp = std::stol(stringList.back());
        beacons.timestamp(timestamp);
        int nBeacons = 0;
        std::string uuid;
        int major=0;
        int minor=0;
        double rssi = -100;
        
        for( iter= stringList.begin(); iter!=stringList.end(); iter++){
            if(i==0){
                if(iter->compare("Beacon") != 0){
                    BOOST_THROW_EXCEPTION(LocException("Log beacon is not correctly formatted. string="+str));
                }
            }
            if(i==1) nBeacons = std::stoi(*iter);
            if(1<i){
                if( stringList.size() == 2*nBeacons + 3 ){
                    // "Beacon",nBeacon,uuid-major-minor,rssi,....,timestamp
                    if(i>1 && i-2 < 2*nBeacons){
                        if(i%2==0){
                            std::string idStr = *iter;
                            auto bId = parseBeaconIdString(idStr);
                            uuid = bId.uuid();
                            major = bId.major();
                            minor = bId.minor();
                        }
                        if(i%2==1){
                            rssi = std::stod(*iter);
                            Beacon b(uuid, major, minor, rssi);
                            beacons.push_back(b);
                        }
                    }
                }else if(stringList.size() == 3*nBeacons + 3) {
                    // "Beacon",nBeacon,major,minor,rssi,....,timestamp
                    if(i>1 && i-2 < 3*nBeacons){
                        if(i%3==2) major = std::stoi(*iter);
                        if(i%3==0) minor = std::stoi(*iter);
                        if(i%3==1){
                            rssi = std::stod(*iter);
                            Beacon b(major, minor, rssi);
                            beacons.push_back(b);
                        }
                    }
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
    
    std::string DataUtils::stringToFile(const std::string& dataStr, const std::string& dir, const std::string& file)
    {
        
        std::string tempPath = dir+"/"+file;
        if (file.empty()) {
            boost::uuids::random_generator gen;
            boost::uuids::uuid u = gen();
            tempPath = dir+"/"+boost::uuids::to_string(u);
        }
        
        std::regex regex( "^(data:([a-z]+/[a-z]+);base64,).+$" );
        std::smatch match;
        
        std::string type;
        std::string data;
        std::string prefix = dataStr.substr(0,50);
        if( regex_match(prefix, match, regex) ) {
            type = match[2];
            type = std::regex_replace( type, std::regex("^[^/]+/(x-)?"), "" );
            long begin = match[1].length();
            
            tempPath += "."+type;
            std::ofstream ofs(tempPath);
            
            unsigned long paddChars = count(dataStr.end()-10, dataStr.end(), '=');
            //std::replace(data.begin(),data.end(),'=','A');
            copy(it_binary_t(dataStr.begin()+begin), it_binary_t(dataStr.end()-paddChars), std::ostream_iterator<char>(ofs));

        } else {
            tempPath += ".txt";
            std::ofstream ofs(tempPath);
            //copy(dataStr.begin(), dataStr.end(), std::ostream_iterator<char>(ofs));
            ofs << dataStr;
            ofs.close();
        }
        
        return tempPath;
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
        return DataUtils::parseSampleCSV(csvLine, false);
    }
    Sample DataUtils::parseSampleCSV(const std::string& csvLine, bool noBeacons) throw(std::invalid_argument) {
        // timestamp,"Beacon",x,y,z,floor,N,uuid1-major1-minor1,rssi1,...,uuidN-majorN-minorN,rssiN
        // or
        // timestamp,"Beacon",x,y,z,floor,N,major1,minor1,rssi1,...,majorN,minorN,rssiN
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
        Sample sample;
        sample.location(location)->timestamp(timestamp);
        
        if (!noBeacons) {
            int major, minor;
            double rssi;
            Beacons beacons;
            beacons.timestamp(timestamp);
            
            std::istringstream iss(buffer);
            std::string newCell;
            std::vector<std::string> csvTokens;
            while( std::getline( iss, newCell, ',' ) ) {
                csvTokens.push_back(newCell);
            }
            if(csvTokens.size() == 2*num ){ // "uuid-major-minor,rssi" format
                for(int i=0; i<num; i++){
                    auto beaconId = parseBeaconIdString(csvTokens[2*i]);
                    rssi = stod(csvTokens[2*i+1]);
                    Beacon b(beaconId, rssi);
                    beacons.push_back(b);
                }
            }else if(csvTokens.size() == 3*num ){ // "major,minor,rssi" format
                for(int i = 0; i < num; i++) {
                    sscanf(buffer, "%d,%d,%lf,%n", &major, &minor, &rssi, &n);
                    buffer += n;
                    beacons.push_back(Beacon(major, minor, rssi));
                }
            }else{
                throw std::invalid_argument("Invalid csv line was found.");
            }
            sample.beacons(beacons);
        }
        return sample;
    }
    
    void DataUtils::csvLocationsToLocations(std::istream& istream, Locations &locations){
        std::string strBuffer;
        while(std::getline(istream, strBuffer)){
            try{
                Location loc = parseLocationCSV(strBuffer);
                locations.push_back(loc);
            } catch (std::invalid_argument e){
                std::cout << "Invalid csv line was found. line=" <<strBuffer << std::endl;
            }
        }
    }
    
    
    void DataUtils::csvSamplesToSamples(std::istream& istream, Samples &samples){
        DataUtils::csvSamplesToSamples(istream, samples, false);
    }
    
    void DataUtils::csvSamplesToSamples(std::istream& istream, Samples &samples, bool noBeacons){
        std::string strBuffer;
        while(std::getline(istream, strBuffer)){
            try{
                Sample sample = parseSampleCSV(strBuffer, noBeacons);
                samples.push_back(sample);
            } catch (std::invalid_argument e){
                std::cout << "Invalid csv line was found. line=" <<strBuffer << std::endl;
                //std::cout << "Header line is found in csv samples." << std::endl;
            }
        }
    }
    
    Samples DataUtils::csvSamplesToSamples(std::istream& istream){
        return DataUtils::csvSamplesToSamples(istream, false);
    }
    
    Samples DataUtils::csvSamplesToSamples(std::istream& istream, bool noBeacons){
        Samples samples;
        std::string strBuffer;
        while(std::getline(istream, strBuffer)){
            try{
                Sample sample = parseSampleCSV(strBuffer, noBeacons);
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
        return samples;
    }
    
    std::string DataUtils::samplesToCsvSamples(const Samples &samples){
        // csv format of sample
        // timestamp,"Beacon",x,y,z,floor,#beacons,major,minor,rssi,...
        std::stringstream ss;
        for(auto& s: samples){
            auto ts = s.timestamp();
            auto loc = s.location();
            auto bs = s.beacons();
            ss << ts <<",Beacon," << loc << "," << bs.size();
            for(auto& b: bs){
                ss << ",";
                ss << b;
            }
            ss << std::endl;
        }
        return ss.str();
    }

    
    
    
    
    BLEBeacon DataUtils::parseBLEBeaconCSV(const std::string& csvLine) throw(...){
        // minimum csv format of BLEBeacon
        // uuid, major, minor, x, y, z, floor
        std::list<std::string> stringList = splitAndTrimCSV(csvLine);
        std::list<std::string>::iterator iter;
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
    
    std::string DataUtils::BLEBeaconsToCSV(const BLEBeacons &bleBeacons){
        // uuid,major,minor,x,y,z,floor,power
        std::stringstream ss;
        for(auto& b: bleBeacons){
            ss << b.uuid() << "," << b.major() << "," << b.minor()
            << "," << b.x() << "," << b.y() << "," << b.z() << "," << b.floor()
            << "," << b.power();
            ss << std::endl;
        }
        return ss.str();
    }
    
    BLEBeacons csvBLEBeaconsToBLEBeaconsNoHeader(std::istream& istream){
        BLEBeacons bleBeacons;
        std::string strBuffer;
        while(std::getline(istream, strBuffer)){
            try{
                BLEBeacon bleBeacon = DataUtils::parseBLEBeaconCSV(strBuffer);
                bleBeacons.push_back(bleBeacon);
            }catch(std::invalid_argument e){
                std::cout << "Invalid csv line was found. line=" <<strBuffer << std::endl;
                //std::cout << "Header line is found in csv BLEBeacons." << std::endl;
            }catch(std::out_of_range& e){
                std::cout << "Invalid csv line was found. line=" <<strBuffer << std::endl;
            }
        }
        if(bleBeacons.size()==0){
            std::cout << "BLEBeacons CSV was not found." << std::endl;
        }
        return bleBeacons;
    }
    
    BLEBeacons csvBLEBeaconsToBLEBeaconsWithHeader(std::istream& istream){
        BLEBeacons bleBeacons;
        std::string strBuffer;
        std::vector<std::string> headerKeys;
        
        while(std::getline(istream, strBuffer)){
            // check header
            std::string headerMain = "uuid,major,minor,x,y,z,floor";
            if(strBuffer.find(headerMain) == 0){
                std::cout << "Header line was found. line=" <<strBuffer << std::endl;
                std::list<std::string> stringList = DataUtils::splitAndTrimCSV(strBuffer);
                std::vector<std::string> stringVector = DataUtils::listToVector(stringList);
                headerKeys = stringVector;
                continue;
            }
            
            // read csv blebeacon
            try{
                std::list<std::string> stringList = DataUtils::splitAndTrimCSV(strBuffer);
                std::vector<std::string> strVec = DataUtils::listToVector(stringList);
                
                std::string uuid = strVec.at(0);
                int major = std::stoi(strVec.at(1));
                int minor = std::stoi(strVec.at(2));
                double x = std::stod(strVec.at(3));
                double y = std::stod(strVec.at(4));
                double z = std::stod(strVec.at(5));
                double floor = std::stod(strVec.at(6));
                BLEBeacon bleBeacon(uuid, major, minor, x, y, z, floor);
                
                // parse optional variables
                if( 7 < headerKeys.size()){ // header exists
                    for(int i = 7; i<headerKeys.size() ; i++){
                        const auto& headerKey = headerKeys.at(i);
                        if(headerKey == "power"){
                            try{
                                bleBeacon.power(std::stod(strVec.at(i)));
                            }catch(const std::invalid_argument& e){
                                std::cout << "invalid value for BLEBeacon power. value=" << strVec.at(i) << std::endl;
                                bleBeacon.power(0.0);
                            }catch(const std::out_of_range& e){
                                std::cout << "out_of_range for BLEBeacon power. line=" << strBuffer << strBuffer << std::endl;
                                bleBeacon.power(0.0);
                            }
                        }
                    }
                }else{
                    // assumes the default header
                    // "uuid,major,minor,x,y,z,floor,power"
                    for(int i = 7; i<strVec.size(); i++){
                        if(i==7){ // power
                            try{
                                bleBeacon.power(std::stod(strVec.at(i)));
                            }catch(const std::invalid_argument& e){
                                std::cout << "invalid value for BLEBeacon power. value=" << strVec.at(i) << std::endl;
                                bleBeacon.power(0.0);
                            }
                        }
                    }
                }
                bleBeacons.push_back(bleBeacon);
            }catch(std::invalid_argument e){
                std::cout << "Invalid csv line was found. line=" <<strBuffer << std::endl;
            }catch(std::out_of_range& e){
                std::cout << "Invalid csv line was found. line=" <<strBuffer << std::endl;
            }
        }
        
        if(bleBeacons.size()==0){
            std::cout << "BLEBeacons CSV was not found." << std::endl;
        }
        
        return bleBeacons;
    }
    
    BLEBeacons DataUtils::csvBLEBeaconsToBLEBeacons(std::istream& istream){
        //return csvBLEBeaconsToBLEBeaconsNoHeader(istream);
        return csvBLEBeaconsToBLEBeaconsWithHeader(istream);
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
