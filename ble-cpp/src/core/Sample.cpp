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
#include <set>
#include <map>
#include "Sample.hpp"

namespace loc{
    
    bool Sample::equalLocation(Sample sample1, Sample sample2){
        Location loc1 = sample1.location();
        Location loc2 = sample2.location();
        return Location::equals(loc1, loc2);
    }
    
    std::vector<std::vector<Sample>> Sample::splitSamplesToConsecutiveSamples(std::vector<Sample> samples){
        
        Sample prevSample = samples.at(0);
        std::vector<std::vector<Sample>> samplesList;
        for(auto iter = samples.begin(); iter!=samples.end(); iter++){
            if(iter==samples.begin()){
                std::vector<Sample> samplesTmp;
                samplesTmp.push_back(*iter);
                samplesList.push_back(samplesTmp);
            }else{
                if(Sample::equalLocation(prevSample, *iter)){
                    samplesList.back().push_back(*iter);
                }else{
                    std::vector<Sample> samplesTmp;
                    samplesTmp.push_back(*iter);
                    samplesList.push_back(samplesTmp);
                }
            }
            prevSample = *iter;
        }
        return samplesList;
    }
    
    Sample Sample::mean(std::vector<Sample> samples){
        
        long timeSum = 0;
        std::vector<Location> locs;
        std::vector<Beacons> beaconsList;
        
        for(Sample smp :samples){
            timeSum += smp.timestamp();
            Location loc = smp.location();
            Beacons beacons = smp.beacons();
            locs.push_back(loc);
            beaconsList.push_back(beacons);
        }
        
        long timestamp = timeSum/samples.size();
        Location locMean = Location::mean(locs);
        Beacons beaconsMean = Beacon::meanBeaconsVector(beaconsList);
        
        Sample sampleMean;
        sampleMean.timestamp(timestamp);
        sampleMean.location(locMean);
        
        Beacons beacons(beaconsMean);
        beacons.timestamp(timestamp);
        sampleMean.beacons(beacons);
        
        return sampleMean;
    }
    
    
    std::vector<Sample> Sample::mean(std::vector<std::vector<Sample>> samplesList){
        std::vector<Sample> sampleMeans;
        for(std::vector<Sample> samples: samplesList){
            Sample smp = Sample::mean(samples);
            sampleMeans.push_back(smp);
        }
        return sampleMeans;
        
    }
    
    std::vector<Sample> Sample::meanUniqueLocations(std::vector<Sample> samples){
        
        std::map<Location, std::vector<Beacons> > locBeaconsVectorMap;
        
        for(Sample smp: samples){
            Location loc = smp.location();
            Beacons beacons = smp.beacons();
            /////std::cout << "loc=" << loc <<std::endl;
            if(locBeaconsVectorMap.count(loc)==0){
                std::vector<Beacons> beaconsVector;
                beaconsVector.push_back(beacons);
                locBeaconsVectorMap[loc] = beaconsVector;
            }else{
                std::vector<Beacons> beaconsVector = locBeaconsVectorMap.at(loc);
                beaconsVector.push_back(beacons);
            }
        }
        
        
        std::vector<std::vector<Sample>> samplesList;

        for(auto iter=locBeaconsVectorMap.begin(); iter!=locBeaconsVectorMap.end(); iter++){
            std::vector<Sample> samples;
            Location loc = iter->first;
            std::vector<Beacons> beaconsVector = iter->second;
            for(Beacons bs: beaconsVector){
                Sample smp;
                smp.timestamp(bs.timestamp());
                smp.location(loc);
                smp.beacons(bs);
                samples.push_back(smp);
            }
            samplesList.push_back(samples);
        }
        
        std::vector<Sample> sampleMeans = mean(samplesList);
        return sampleMeans;
    }
    
    std::vector<Location> Sample::extractUniqueLocations(const Samples& samples){
        std::set<Location> locationSet;
        
        for(const Sample& smp: samples){
            Location loc = smp.location();
            locationSet.insert(loc);
        }
        std::vector<Location> locs;
        for(Location loc: locationSet){
            locs.push_back(loc);
        }
        
        return locs;
    }
    
    Samples Sample::filterUnregisteredBeacons(const Samples &samples, const BLEBeacons& bleBeacons){
        Samples smps;
        std::map<long, int> indexMap = BLEBeacon::constructBeaconIdToIndexMap(bleBeacons);
        for(Sample s: samples){
            Beacons bs = s.beacons();
            Beacons bsnew;
            bsnew.timestamp(bs.timestamp());
            for(Beacon b: bs){
                long id = b.id();
                if(indexMap.count(id)>0){
                    bsnew.push_back(b);
                }
            }
            if(bsnew.size()>0){
                s.beacons(bsnew);
                smps.push_back(s);
            }
        }
        if(smps.size()==0){
            std::stringstream ss1;
            ss1 << "Registered beacons(major,minor) = ";
            for(const BLEBeacon& ble: bleBeacons){
                ss1 << "(" << ble.major() << "," << ble.minor() <<"),";
            }
            throw new std::runtime_error("No sample contains beacon signals from the registered beacons");
        }
        return smps;
    }
    
}