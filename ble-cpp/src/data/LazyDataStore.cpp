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

#include "LazyDataStore.hpp"
#include "DataUtils.hpp"

namespace loc{
    
    LazyDataStore& samples(Samples samples);
    LazyDataStore& bleBeacons(BLEBeacons bleBeacons);
    LazyDataStore& building(Building building);
    
    LazyDataStore& LazyDataStore::samplesFilePathes(std::vector<std::string> samplesFilePathes){
        mSamplesFilePathes = samplesFilePathes;
        return *this;
    }
    
    LazyDataStore& LazyDataStore::bleBeaconsFilePathes(std::vector<std::string> bleBeaconsFilePathes){
        mBLEBeaconsFilePathes = bleBeaconsFilePathes;
        return *this;
    }
    
    LazyDataStore& LazyDataStore::building(Building building){
        mBuilding = building;
        return *this;
    }
    
    Samples LazyDataStore::getSamples() const{
        Samples samples;
        for(std::string trainPath: mSamplesFilePathes){
            std::ifstream  istream(trainPath);
            Samples samplesTmp = DataUtils::csvSamplesToSamples(istream);
            samples.insert(samples.end(), samplesTmp.begin(), samplesTmp.end());
        }
        return samples;
    }
    
    BLEBeacons LazyDataStore::getBLEBeacons() const{
        BLEBeacons bleBeacons;
        for(std::string bleBeaconPath : mBLEBeaconsFilePathes){
            std::ifstream bleBeaconIStream(bleBeaconPath);
            BLEBeacons bleBeaconsTmp = DataUtils::csvBLEBeaconsToBLEBeacons(bleBeaconIStream);
            bleBeacons.insert(bleBeacons.end(), bleBeaconsTmp.begin(), bleBeaconsTmp.end());
        }
        return bleBeacons;
    }
    
    Building LazyDataStore::getBuilding() const{
        return mBuilding;
    }
    
}