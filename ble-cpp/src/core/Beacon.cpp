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

#include "Beacon.hpp"
#include <sstream>
#include <map>
#include <set>
#include <algorithm>

#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid_io.hpp>

namespace loc{
    
    Beacon::Beacon(const BeaconId & id, double rssi){
        id_ = id;
        rssi_ = rssi;
    }
    
    Beacon::Beacon(int major, int minor, double rssi){
        std::string uuid = "";
        id_ = BeaconId(uuid, major, minor);
        rssi_ = rssi;
    }
    
    Beacon::Beacon(const std::string& uuid, int major, int minor, double rssi){
        id_ = BeaconId(uuid, major, minor);
        rssi_ = rssi;
    }
    
    Beacon::~Beacon(){}

    const std::string& Beacon::uuid() const{
        return id_.uuid();
    }

    Beacon& Beacon::uuid(const std::string& uuid){
        id_ = BeaconId(uuid, id_.major(), id_.minor());
        return *this;
    }

    int Beacon::major() const{
        return id_.major();
    }
    
    Beacon& Beacon::major(int major){
        id_ = BeaconId(id_.uuid(), major, id_.minor());
        return *this;
    }
    
    int Beacon::minor() const{
        return id_.minor();
    }
    
    Beacon& Beacon::minor(int minor){
        id_ = BeaconId(id_.uuid(), id_.major(), minor);
        return *this;
    }
    
    double Beacon::rssi() const{
        return rssi_;
    }
    
    Beacon& Beacon::rssi(double rssi){
        this->rssi_ = rssi;
        return *this;
    }
    
    const BeaconId& Beacon::id() const{
        return id_;
    }
    
    std::string Beacon::toString(){
        std::stringstream stream;
        stream << id_.toString() << "," << rssi_;
        return stream.str();
    }
    
    template<class Tbeacons>
    Tbeacons Beacon::meanBeaconsVector(std::vector<Tbeacons> beaconsVector){
        std::map<BeaconId, int> id_count;
        std::map<BeaconId, double> id_rssiSum;
        
        for(int i=0; i<beaconsVector.size(); i++){
            Tbeacons beacons = beaconsVector.at(i);
            for(int j=0; j<beacons.size(); j++){
                Beacon b = beacons.at(j);
                const auto& id = b.id();
                if(id_count.count(id)==0){
                    id_count[id]=1;
                    id_rssiSum[id] = b.rssi();
                }else{
                    id_count[id]+=1;
                    id_rssiSum[id] += b.rssi();
                }
            }
        }
        
        Tbeacons beaconsAveraged;
        for(auto iter=id_count.begin(); iter!=id_count.end(); iter++){
            const auto& id = iter->first;
            double rssi = id_rssiSum[id]/iter->second;
            Beacon b(id, rssi);
            beaconsAveraged.push_back(b);
        }
        return beaconsAveraged;
        
    }
    
    class BeaconId::Impl{
    public:
        static const long large_value = 100000;
    };
    
    Beacons Beacon::filter(const Beacons& beacons, double minRssi, double maxRssi){
        Beacons beaconsFiltered(beacons);
        beaconsFiltered.clear();
        for(Beacon b: beacons){
            double rssi = b.rssi();
            if( minRssi<rssi && rssi<maxRssi){
                beaconsFiltered.push_back(b);
            }
        }
        return beaconsFiltered;
    }
    
    struct LessRSSI{
        bool operator()(const Beacon& b1, const Beacon& b2){
            return b1.rssi() < b2.rssi();
        }
    };
    
    Beacons Beacon::sortByRssi(const Beacons& beacons){
        Beacons beaconsSorted(beacons);
        std::sort(beaconsSorted.begin(), beaconsSorted.end(), LessRSSI());
        return beaconsSorted;
    }
    
    std::ostream& operator<<(std::ostream&os, const Beacon& beacon){
        os << beacon.major() <<"," << beacon.minor() <<"," << beacon.rssi();
        return os;
    }
    
    template
    Beacons Beacon::meanBeaconsVector<Beacons>(std::vector<Beacons> beaconsVector);
    
    template
    std::vector<Beacon> Beacon::meanBeaconsVector<std::vector<Beacon>>(std::vector<std::vector<Beacon>> beaconsVector);
    
    
    // Implementation of Beacons class
    
    Beacons& Beacons::timestamp(long timestamp){
        mTimestamp = timestamp;
        return *this;
    }
    
    long Beacons::timestamp() const{
        return mTimestamp;
    }
    

    
    // Beacon Id class
    
    BeaconId::BeaconId(const std::string& uuid, int major, int minor){
        uuid_ = uuid;
        major_ = major;
        minor_ = minor;
        setuuid(uuid);
    }
    
    template<class Archive>
    void BeaconId::serialize(Archive& ar, std::uint32_t const version){
        ar(CEREAL_NVP(uuid_));
        ar(CEREAL_NVP(major_));
        ar(CEREAL_NVP(minor_));
        setuuid(uuid_);
    }
    
    template void BeaconId::serialize<cereal::JSONOutputArchive> (cereal::JSONOutputArchive& archive, std::uint32_t const version);
    template void BeaconId::serialize<cereal::JSONInputArchive> (cereal::JSONInputArchive& archive, std::uint32_t const version);
    template void BeaconId::serialize<cereal::PortableBinaryOutputArchive> (cereal::PortableBinaryOutputArchive& archive, std::uint32_t const version);
    template void BeaconId::serialize<cereal::PortableBinaryInputArchive> (cereal::PortableBinaryInputArchive& archive, std::uint32_t const version);
    
    void BeaconId::setuuid(const std::string& uuid){
        if(!uuid.empty()){
            buuid_ = boost::uuids::string_generator()(uuid);
        }else{
            buuid_ = boost::uuids::nil_uuid();
            uuid_ = boost::lexical_cast<std::string>(buuid_);
        }
    }
    
    const std::string& BeaconId::uuid() const{
        return uuid_;
    }
    
    const boost::uuids::uuid& BeaconId::buuid() const{
        return buuid_;
    }
    
    int BeaconId::major() const{
        return major_;
    }
    
    int BeaconId::minor() const{
        return minor_;
    }
    
    bool BeaconId::operator==(const BeaconId& id) const{
        if(this->buuid_.is_nil() || id.buuid_.is_nil()){
            return this->major_ == id.major_ && this->minor_ == id.minor_;
        }else{
            return this->major_ == id.major_ && this->minor_ == id.minor_ && this->buuid_ == id.buuid_;
        }
    }
    
    bool BeaconId::operator<(const BeaconId& id) const{
        if(this->buuid_.is_nil() || id.buuid_.is_nil()){
            if (this->major_ == id.major_){
                return this->minor_ < id.minor_;
            }else{
                return this->major_ < id.major_;
            }
        }else{
            if( this->buuid_ == id.buuid_ ){
                if (this->major_ == id.major_){
                    return this->minor_ < id.minor_;
                }else{
                    return this->major_ < id.major_;
                }
            }else{
                return this->buuid_ < id.buuid_;
            }
        }
    }
    
    const std::string BeaconId::toString() const{
        std::stringstream ss;
        ss << uuid() << "-" << major() << "-" << minor();
        return ss.str();
    }
    
    BeaconId BeaconId::convertLongIdToId(long long_id){
        int major = convertIdToMajor(long_id);
        int minor = convertIdToMinor(long_id);
        BeaconId id;
        id.major_ = major;
        id.minor_ = minor;
        return id;
    }
    
    long BeaconId::convertToLongId(const BeaconId& id){
        return Impl::large_value * id.major() + id.minor();
    }
    
    // Encoding rule
    // id = large_value*major + minor
    // major = id/large_value;
    // minor = id%large_value;
    
    /*
     BeaconId Beacon::convertMajorMinorToId(int major, int minor){
     long large_value = Impl::large_value;
     long id = major*large_value + minor;
     return id;
     }
     */
    
    int BeaconId::convertIdToMajor(long id){
        long large_value = Impl::large_value;
        int major = static_cast<int>(id/large_value);
        return major;
    }
    
    int BeaconId::convertIdToMinor(long id){
        long large_value = Impl::large_value;
        int minor = static_cast<int>(id%large_value);
        return minor;
    }
    
    
}
