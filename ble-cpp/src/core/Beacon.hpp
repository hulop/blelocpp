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

#ifndef Beacon_hpp
#define Beacon_hpp

#include <stdio.h>
#include <iostream>
#include <vector>
#include <cctype>
#include <clocale>
#include <algorithm>

#include "SerializeUtils.hpp"

namespace loc{
    
    class Beacon;
    class Beacons;
    
    class Beacon{
    
    private:
        std::string uuid_;
        int major_;
        int minor_;
        double rssi_;
        class Impl;
        
    public:
        class Id;
        
        Beacon(int major, int minor, double rssi);
        Beacon(std::string uuid, int major, int minor, double rssi);
        ~Beacon();
        
        std::string uuid() const;
        int major() const;
        int minor() const;
        double rssi() const;
        
        Id id() const;
        
        Beacon& uuid(std::string uuid);
        Beacon& major(int major);
        Beacon& minor(int minor);
        Beacon& rssi(double rssi);
       
        std::string toString();
        
        template<class Tbeacons>
        static Tbeacons meanBeaconsVector(std::vector<Tbeacons> beaconsVector);
        
        static Beacons filter(const Beacons& beacons, double minRssi, double maxRssi);
        static Beacons sortByRssi(const Beacons& beacons);
        
        friend std::ostream& operator<<(std::ostream&os, const Beacon& beacon);
    };
    
    // Beacons class
    class Beacons : public std::vector<Beacon>{
    private:
        long mTimestamp;
    public:
        Beacons() = default;
        ~Beacons() = default;
        Beacons& timestamp(long timestamp);
        long timestamp() const;
    };
    
    // Beacon Config class
    class BeaconConfig{
        constexpr const static double maxRssi_ = -1;
        constexpr const static double minRssi_ = -100;
    public:
        static double maxRssi(){
            return maxRssi_;
        }
        static double minRssi(){
            return minRssi_;
        }
        
        static bool checkInRssiRange(const Beacon& beacon){
            if(minRssi_ < beacon.rssi() && beacon.rssi() < maxRssi_){
                return true;
            }else{
                return false;
            }
        }
    };
    
    // Beacon Id class
    class Beacon::Id{
        std::string uuid_;
        int major_;
        int minor_;
        
        static int convertIdToMajor(long id);
        static int convertIdToMinor(long id);
        
    public:
        Id() = default;
        Id(const std::string& uuid, int major, int minor);
        
        std::string uuid() const;
        int major() const;
        int minor() const;
        
        bool operator==(const Id& id) const;
        bool operator<(const Id& id) const;
        
        static Id convertLongIdToId(long long_id); // for backward compatibility
        
        template<class Archive>
        void serialize(Archive& ar, std::uint32_t const version){
            ar(CEREAL_NVP(uuid_));
            ar(CEREAL_NVP(major_));
            ar(CEREAL_NVP(minor_));
        }
    };

}

// assign version
CEREAL_CLASS_VERSION(loc::Beacon::Id, 0);
#endif /* Beacon_hpp */
