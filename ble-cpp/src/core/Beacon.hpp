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

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>

#include "SerializeUtils.hpp"

namespace loc{
    
    class BeaconId{
        std::string uuid_;
        boost::uuids::uuid buuid_ = boost::uuids::nil_uuid();
        int major_;
        int minor_;
        
        class Impl;
        void setuuid(const std::string& uuid);
        static int convertIdToMajor(long id);
        static int convertIdToMinor(long id);
    public:
        BeaconId() = default;
        BeaconId(const std::string& uuid, int major, int minor);
        
        const std::string& uuid() const;
        const boost::uuids::uuid& buuid() const;
        int major() const;
        int minor() const;
        
        bool operator==(const BeaconId& id) const;
        bool operator<(const BeaconId& id) const;
        
        static BeaconId convertLongIdToId(long long_id); // for backward compatibility
        
        template<class Archive>
        void serialize(Archive& ar, std::uint32_t const version);
    };
    
    
    
    class Beacon;
    class Beacons;
    
    class Beacon{
    private:
        BeaconId id_;
        double rssi_;
    public:
        
        Beacon(int major, int minor, double rssi);
        Beacon(const std::string& uuid, int major, int minor, double rssi);
        ~Beacon();
        
        const std::string& uuid() const;
        int major() const;
        int minor() const;
        double rssi() const;
        
        const BeaconId& id() const;
        
        Beacon& uuid(const std::string& uuid);
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
    
}

// assign version
CEREAL_CLASS_VERSION(loc::BeaconId, 0);
#endif /* Beacon_hpp */
