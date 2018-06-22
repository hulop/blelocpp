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

#include "CnnFileUtil.hpp"

#include <fstream>
#include <iostream>
#include <picojson.h>
#include <boost/algorithm/string.hpp>

namespace {
    const picojson::value &get(const picojson::value::object &obj, std::string key){
        auto itr = obj.find(key);
        if (itr != obj.end()) {
            return itr->second;
        }
        throw "not found";
    }
    
    const std::string &getString(const picojson::value::object &obj, std::string key) {
        auto& value = get(obj, key);
        if (value.is<std::string>()) {
            return value.get<std::string>();
        }
        throw "non string value";
    }
    double getDouble(const picojson::value::object &obj, std::string key) {
        auto& value = get(obj, key);
        if (value.is<double>()) {
            return value.get<double>();
        }
        throw "non double value";
    }
    const picojson::value::object &getObject(const picojson::value::object &obj, std::string key) {
        auto& value = get(obj, key);
        if (value.is<picojson::value::object>()) {
            return value.get<picojson::value::object>();
        }
        throw "non object value";
    }
    const picojson::value::array &getArray(const picojson::value::object &obj, std::string key) {
        auto& value = get(obj, key);
        if (value.is<picojson::value::array>()) {
            return value.get<picojson::value::array>();
        }
        throw "non array value";
    }
}

namespace loc {
    
    std::list<std::string> CnnFileUtil::splitSSV(const std::string & str){
        std::list<std::string> stringList;
        std::string delim (" ");
        boost::split(stringList, str, boost::is_any_of(delim));
        return stringList;
    }
    
    std::list<std::string> CnnFileUtil::splitAndTrimSSV(const std::string & str){
        std::list<std::string> stringList = splitSSV(str);
        std::list<std::string>::iterator iter;
        for( iter= stringList.begin(); iter!=stringList.end(); iter++){
            boost::trim(*iter);
        }
        return stringList;
    }
    
    std::map<std::pair<int,int>,int> CnnFileUtil::parseBeaconSettingFile(const std::string& beaconFilePath){
        std::map<std::pair<int,int>,int> beaconIndex;
        
        std::ifstream beaconfile(beaconFilePath);
        if (beaconfile.is_open()) {
            std::string line;
            std::getline(beaconfile, line);
            
            std::list<std::string> tokens = CnnFileUtil::splitAndTrimSSV(line);
            assert(tokens.size()==1);
            
            // read beacon list and generate map
            int nBeacon = stoi(*tokens.begin());
            for (int i = 0; i<nBeacon; i++) {
                getline(beaconfile,line);
                tokens = CnnFileUtil::splitAndTrimSSV(line);
                assert(tokens.size()==2);
                std::list<std::string>::iterator iter = tokens.begin();
                int major = stoi(*iter);
                iter++;
                int minor = stoi(*iter);
                beaconIndex.insert(std::make_pair(std::make_pair(major,minor),i));
                std::cout << "major=" << major << ", minor=" << minor << ", index=" << i << std::endl;
            }
        }
        beaconfile.close();
        
        assert(beaconIndex.size()>0);
        
        return beaconIndex;
    }
    
    void CnnFileUtil::parseBeaconString(const std::string &line, std::map<std::pair<int,int>,int> &beaconIndex, cv::Mat &rssi){

        std::vector<std::string> tokens;
        boost::split(tokens, line, boost::is_any_of(","));
        int tokenIndex = 0;
        tokenIndex++; // timestamp
        tokenIndex++; // "Beacon"
        tokenIndex += 4; // skip another 4 empty spaces
        int nSignal = stoi(tokens[tokenIndex++]); // number of signal detected
        for (int j=0; j<nSignal; j++) {
            int major = stoi(tokens[tokenIndex++]);
            int minor = stoi(tokens[tokenIndex++]);
            int strength = stoi(tokens[tokenIndex++]);
            if (strength==0) {
                // raw RSSI value 0 is invalid, set -100
                strength = -100;
            }
            
            std::pair<int,int> majorMinorPair = std::make_pair(major,minor);
            std::map<std::pair<int,int>,int>::iterator rssiInd = beaconIndex.find(majorMinorPair);
            assert(rssiInd != beaconIndex.end());
            
            rssi.at<int>(rssiInd->second) = strength;
        }
    }
    
    void CnnFileUtil::parseCnnSettingJsonFile(const std::string& cnnSettingFilePath,
                                              const std::string& cnnModelParentDirPath,
                                              loc::ImageLocalizeMode imageLocalizeMode,
                                              std::map<int, std::string>& cnnModelFileDict,
                                              std::map<int, std::string>& beaconSettingFileDict) {
        std::ifstream file;
        file.open(cnnSettingFilePath, std::ios::in);
        std::istreambuf_iterator<char> input(file);
        
        picojson::value v;
        std::string err;
        picojson::parse(v, input, std::istreambuf_iterator<char>(), &err);
        if (!err.empty()) {
            throw err+" with reading "+cnnSettingFilePath;
        }
        if (!v.is<picojson::object>()) {
            throw "invalid JSON";
        }
        file.close();
        std::cerr << "parse JSON: " << std::endl;
        
        picojson::value::object& json = v.get<picojson::object>();
        try{
            cnnModelFileDict.clear();
            beaconSettingFileDict.clear();
            
            auto& cnnModels = getArray(json, "CnnModels");
            for(int i = 0; i < cnnModels.size(); i++) {
                auto& cnnModel = cnnModels.at(i).get<picojson::value::object>();
                int floor = (int)getDouble(cnnModel, "Floor");
                
                picojson::value::object mode;
                if (imageLocalizeMode==ImageLocalizeMode::IMAGE) {
                    mode = getObject(cnnModel, "ImageCnn");
                } else if (imageLocalizeMode==ImageLocalizeMode::BEACON) {
                    mode = getObject(cnnModel, "BeaconCnn");
                } else if (imageLocalizeMode==ImageLocalizeMode::IMAGE_BEACON) {
                    mode = getObject(cnnModel, "ImageBeaconCnn");
                } else {
                    assert(false);
                }
                
                std::string modelPath = getString(mode, "ModelPath");
                std::string beaconSettingPath = getString(mode, "BeaconSettingPath");
                assert(modelPath.length()>0 && beaconSettingPath.length()>0);
                
                std::cerr << "Floor : " << floor << std::endl;
                std::cerr << "ModelPath : " << modelPath << std::endl;
                std::cerr << "BeaconSettingPath : " << beaconSettingPath << std::endl;
                
                std::string modelFullPath = cnnModelParentDirPath + "/" + modelPath;
                std::string beaconSettingFullPath = cnnModelParentDirPath + "/" + beaconSettingPath;
                
                cnnModelFileDict[floor] = modelFullPath;
                beaconSettingFileDict[floor] = beaconSettingFullPath;
            }
        }catch(const char* ch){
            std::cerr << "CNN settings have not been loaded : " << ch << std::endl;
        }
    }

}

