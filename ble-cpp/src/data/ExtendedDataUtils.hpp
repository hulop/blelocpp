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

#ifndef ExtendedDataUtils_hpp
#define ExtendedDataUtils_hpp

#include <stdio.h>
#include <string>
#include <sstream>
#include "picojson.h"

#include "bleloc.h"
#include "DataUtils.hpp"

namespace loc{
    class ExtendedDataUtils{
        public:
        
        static picojson::array beaconIdRssiStatsToJSONArray(const std::map<long, NormalParameter>& beaconIdRssiStats){
            picojson::array array;
            for(auto it=beaconIdRssiStats.begin(); it!=beaconIdRssiStats.end(); ++it){
                picojson::object obj;
                long id = it->first;
                auto maj = Beacon::convertIdToMajor(id);
                auto min = Beacon::convertIdToMinor(id);
                double pred = it->second.mean();
                double stdev = it->second.stdev();
                obj.insert(std::make_pair("major", picojson::value((double) maj)));
                obj.insert(std::make_pair("minor", picojson::value((double) min)));
                obj.insert(std::make_pair("mean", picojson::value(pred)));
                obj.insert(std::make_pair("stdev", picojson::value(stdev)));
                array.push_back(picojson::value(obj));
            }
            return array;
        }
        
        static picojson::object predictionDataToJSONObject(const State& state, const Beacons& beacons, const std::map<long, NormalParameter> beaconIdRssiStats){
            picojson::object obj;
            picojson::object stateObj = DataUtils::stateToJSONObject(state);
            picojson::array beaconsArray = DataUtils::beaconsToJSONArray(beacons);
            picojson::array statsArray = beaconIdRssiStatsToJSONArray(beaconIdRssiStats);
            
            obj.insert(std::make_pair("state", picojson::value(stateObj)));
            obj.insert(std::make_pair("beacons", picojson::value(beaconsArray)));
            obj.insert(std::make_pair("predictions", picojson::value(statsArray)));
            
            return obj;
        }
    };
}

#endif /* ExtendedDataUtils_hpp */
