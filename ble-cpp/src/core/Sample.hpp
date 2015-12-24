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

#ifndef Sample_hpp
#define Sample_hpp

#include <stdio.h>
#include <vector>
#include <sstream>
#include "Location.hpp"
#include "Beacon.hpp"

namespace loc{
    
    class Sample;
    using Samples = std::vector<Sample>;
    
    class Sample{
    private:
        long timestamp_;
        Location location_;
        Beacons beacons_;
        
    public:
        Sample() = default;
        ~Sample() = default;
        
        
        long timestamp(){
            return timestamp_;
        }
        
        Sample* timestamp(long timestamp){
            timestamp_ = timestamp;
            return this;
        }
        
        Location location(){
            return location_;
        }
        
        Sample* location(Location location){
            location_ = location;
            return this;
        }
        
        Beacons beacons(){
            return beacons_;
        }
        
        Sample* beacons(Beacons beacons){
            beacons_ = beacons;
            return this;
        }
        
        std::string toString(){
            std::stringstream str ;
            str << timestamp() << ",";
            str << location_.toString();
            for(Beacon b: beacons_){
                str << ",";
                str << b.toString();
            }
            return str.str();
        }
        
        
        static bool equalLocation(Sample sample1, Sample sample2);
        
        static std::vector<std::vector<Sample>> splitSamplesToConsecutiveSamples(std::vector<Sample> samples);
        
        static Sample mean(std::vector<Sample> samples);
        static std::vector<Sample> mean(std::vector<std::vector<Sample>> samplesList);
        
        static std::vector<Sample> meanUniqueLocations(std::vector<Sample> samples);
        
        static std::vector<Location> extractUniqueLocations(const Samples& samples);
        
    };
    
    
    
}

#endif /* Sample_hpp */
