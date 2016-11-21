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

#ifndef BLEBeacon_hpp
#define BLEBeacon_hpp

#include <stdio.h>
#include <vector>
#include <map>
#include <iostream>
#include <sstream>

#include <boost/bimap/bimap.hpp>

#include "Location.hpp"
#include "Beacon.hpp"
#include "LocException.hpp"

namespace loc{
    class BLEBeacon;
    using BLEBeacons = std::vector<BLEBeacon>;

    // BLEBeacon class
    class BLEBeacon : public Location{
    protected:
        std::string uuid_;
        int major_;
        int minor_;
    public:
        
        template<class Archive>
        void serialize(Archive& ar);
        
        BLEBeacon() = default;
        ~BLEBeacon() = default;
        
        BLEBeacon(std::string uuid, int major, int minor, double x, double y, double z, double floor){
            this->x(x);
            this->y(y);
            this->z(z);
            this->floor(floor);
            this->uuid(uuid);
            this->major(major);
            this->minor(minor);
        }
        
        BLEBeacon* uuid(std::string uuid){
            uuid_ = uuid;
            return this;
        }
        
        BLEBeacon* major(int major){
            major_ = major;
            return this;
        }
        
        BLEBeacon* minor(int minor){
            minor_ = minor;
            return this;
        }
        
        std::string uuid() const{
            return uuid_;
        }
        
        int major() const{
            return major_;
        }
        
        int minor() const{
            return minor_;
        }
        
        long id() const{
            return Beacon::convertMajorMinorToId(major(), minor());
        }
        
        std::string toString() const{
            std::stringstream strstream;
            strstream << uuid_  << "," << major_ << "," << minor_ << "," << this->Location::toString();
            return strstream.str();
        }
        
        template<class Tbeacon>
        static bool checkNoDuplication(const std::vector<Tbeacon>& beacons);
        
        template<class Tbeacon>
        static std::map<long, int> constructBeaconIdToIndexMap(std::vector<Tbeacon> beacons);
        
        template<class Tbeacon>
        static boost::bimaps::bimap<long, int> constructBeaconIdToIndexBimap(std::vector<Tbeacon> beacons);
        
        template<class Tin, class Tout>
        static Tout find(const Tin& beacon, const std::vector<Tout>& beacons);
        
        friend std::ostream& operator<<(std::ostream& os, const BLEBeacon& bleBeacon);
    };
    
    
    
    // Definition of template method
    template<class Tbeacon>
    bool BLEBeacon::checkNoDuplication(const std::vector<Tbeacon>& beacons){
        std::set<long> ids;
        bool flag = true;
        for(Tbeacon b: beacons){
            long id = Beacon::convertMajorMinorToId(b.major(), b.minor());
            if(ids.count(id)>0){
                std::stringstream ss;
                ss << "BLEBeacon(major=" << b.major() <<", minor=" << b.minor() << ") is duplicated";
                flag = false;
                BOOST_THROW_EXCEPTION(LocException(ss.str()));
            }
            ids.insert(id);
        }
        return flag;
    }
    
    template<class Tbeacon>
    std::map<long, int> BLEBeacon::constructBeaconIdToIndexMap(std::vector<Tbeacon> beacons){
        try{
            checkNoDuplication(beacons);
        }catch(LocException& ex){
            ex << boost::error_info<struct err_info, std::string>("Found duplication of beacon identifiers.");
            throw ex;
        }
        std::map<long, int> beaconIdIndexMap;
        int i = 0;
        for(Tbeacon b: beacons){
            int major = b.major();
            int minor = b.minor();
            long id = Beacon::convertMajorMinorToId(major, minor);
            beaconIdIndexMap[id] = i;
            i++;
        }
        return beaconIdIndexMap;
    }
    
    template<class Tbeacon>
    boost::bimaps::bimap<long, int> BLEBeacon::constructBeaconIdToIndexBimap(std::vector<Tbeacon> beacons){
        
        typedef boost::bimaps::bimap<long, int> bimap_type;
        typedef bimap_type::value_type bimap_value_type;
        
        bimap_type beaconIdIndexMap;
        
        int i = 0;
        for(Tbeacon b: beacons){
            int major = b.major();
            int minor = b.minor();
            long id = Beacon::convertMajorMinorToId(major, minor);
            beaconIdIndexMap.insert( bimap_value_type(id, i) );
            i++;
        }
        return beaconIdIndexMap;
    }
    
    template<class Tin, class Tout>
    Tout BLEBeacon::find(const Tin& beacon, const std::vector<Tout>& beacons){
        long id = beacon.id();
        for(const Tout& b: beacons){
            if(id == b.id()){
                return b;
            }
        }
        BOOST_THROW_EXCEPTION(LocException("input beacon was not found in beacon list."));
    }
}

#endif /* BLEBeacon_hpp */
