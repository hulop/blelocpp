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
        BeaconId id_;
        double power_ = 0.0;
    public:
        
        BLEBeacon() = default;
        ~BLEBeacon() = default;
        
        BLEBeacon(const std::string& uuid, int major, int minor, double x, double y, double z, double floor);
        BLEBeacon(const std::string& uuid, int major, int minor, double x, double y, double z, double floor, double power);
        
        const BeaconId& id() const;
        BLEBeacon& uuid(const std::string& uuid);
        BLEBeacon& major(int major);
        BLEBeacon& minor(int minor);
        const std::string& uuid() const;
        int major() const;
        int minor() const;
        std::string toString() const;
        BLEBeacon& power(double power);
        double power() const;
        
        template<class Archive> void serialize(Archive& ar);
        
        static Beacons filter(const Beacons& beacons, const BLEBeacons& blebeacons);
        
        template<class Tbeacon>
        static bool checkNoDuplication(const std::vector<Tbeacon>& beacons);
        
        template<class Tbeacon>
        static std::map<BeaconId, int> constructBeaconIdToIndexMap(std::vector<Tbeacon> beacons);
        
        template<class Tbeacon>
        static boost::bimaps::bimap<BeaconId, int> constructBeaconIdToIndexBimap(std::vector<Tbeacon> beacons);
        
        template<class Tin, class Tout>
        static Tout find(const Tin& beacon, const std::vector<Tout>& beacons);
        
        friend std::ostream& operator<<(std::ostream& os, const BLEBeacon& bleBeacon);

        template<class Tbeacon>
        static size_t countUUID(const std::vector<Tbeacon>& beacons);
    };
    
    
    
    // Definition of template method
    template<class Tbeacon>
    bool BLEBeacon::checkNoDuplication(const std::vector<Tbeacon>& beacons){
        std::set<BeaconId> ids;
        bool flag = true;
        std::stringstream ss;
        int count = 0;
        for(Tbeacon b: beacons){
            const auto& id = b.id();
            if(ids.count(id)>0){
                ss << "BLEBeacon(" << b.uuid() << "=" << b.major() <<"-" << b.minor() << ") is duplicated" << std::endl ;
                flag = false;
                count += 1;
            }
            ids.insert(id);
        }
        if(!flag){
            std::stringstream ss2;
            ss2 << "Total " << count << " registered beacons are duplicated." << std::endl;
            ss2 << ss.str();
            BOOST_THROW_EXCEPTION(LocException(ss2.str()));
        }
        return flag;
    }
    
    template<class Tbeacon>
    std::map<BeaconId, int> BLEBeacon::constructBeaconIdToIndexMap(std::vector<Tbeacon> beacons){
        try{
            checkNoDuplication(beacons);
        }catch(LocException& ex){
            ex << boost::error_info<struct err_info, std::string>("Found duplication of beacon identifiers.");
            throw ex;
        }
        std::map<BeaconId, int> beaconIdIndexMap;
        int i = 0;
        for(Tbeacon b: beacons){
            const auto& id = b.id();
            beaconIdIndexMap[id] = i;
            i++;
        }
        return beaconIdIndexMap;
    }
    
    template<class Tbeacon>
    boost::bimaps::bimap<BeaconId, int> BLEBeacon::constructBeaconIdToIndexBimap(std::vector<Tbeacon> beacons){
        
        typedef boost::bimaps::bimap<BeaconId, int> bimap_type;
        typedef bimap_type::value_type bimap_value_type;
        
        bimap_type beaconIdIndexMap;
        
        int i = 0;
        for(Tbeacon b: beacons){
            const auto& id = b.id();
            beaconIdIndexMap.insert( bimap_value_type(id, i) );
            i++;
        }
        return beaconIdIndexMap;
    }
    
    template<class Tin, class Tout>
    Tout BLEBeacon::find(const Tin& beacon, const std::vector<Tout>& beacons){
        const auto& id = beacon.id();
        for(const Tout& b: beacons){
            if(id == b.id()){
                return b;
            }
        }
        BOOST_THROW_EXCEPTION(LocException("input beacon was not found in beacon list."));
    }

    template<class Tbeacon>
    size_t BLEBeacon::countUUID(const std::vector<Tbeacon>& beacons){
        std::set<boost::uuids::uuid> uuidset;
        for(const Tbeacon& b: beacons){
            uuidset.insert(b.id().buuid());
        }
        return uuidset.size();
    }
}

#endif /* BLEBeacon_hpp */
