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

namespace loc{
    
    Beacon::Beacon(int major, int minor, double rssi){
        major_ = major;
        minor_ = minor;
        rssi_ = rssi;
    }
    
    Beacon::~Beacon(){}
    
    int Beacon::major() const{
        return major_;
    }
    
    Beacon& Beacon::major(int major){
        this->major_ = major;
        return *this;
    }
    
    int Beacon::minor() const{
        return minor_;
    }
    
    Beacon& Beacon::minor(int minor){
        this->minor_ = minor;
        return *this;
    }
    
    double Beacon::rssi() const{
        return rssi_;
    }
    
    Beacon& Beacon::rssi(double rssi){
        this->rssi_ = rssi;
        return *this;
    }
    
    long Beacon::id() const{
        return Beacon::convertMajorMinorToId(major_, minor_);
    }
    
    std::string Beacon::toString(){
        std::stringstream stream;
        stream << major_ << "," << minor_ << "," << rssi_;
        return stream.str();
    }
    
    template<class Tbeacons>
    Tbeacons Beacon::meanBeaconsVector(std::vector<Tbeacons> beaconsVector){
        std::map<long, int> id_count;
        std::map<long, double> id_rssiSum;
        
        for(int i=0; i<beaconsVector.size(); i++){
            Tbeacons beacons = beaconsVector.at(i);
            for(int j=0; j<beacons.size(); j++){
                Beacon b = beacons.at(j);
                long id = convertMajorMinorToId(b.major(), b.minor());
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
            long id = iter->first;
            int major = convertIdToMajor(id);
            int minor = convertIdToMinor(id);
            
            double rssi = id_rssiSum[id]/iter->second;
            Beacon b(major,minor,rssi);
            beaconsAveraged.push_back(b);
        }
        return beaconsAveraged;
        
    }
    
    class Beacon::Impl{
    public:
        static const long large_value = 100000;
    };
    
    // Encoding rule
    // id = large_value*major + minor
    // major = id/large_value;
    // minor = id%large_value;
    long Beacon::convertMajorMinorToId(int major, int minor){
        long large_value = Impl::large_value;
        long id = major*large_value + minor;
        return id;
    }
    
    int Beacon::convertIdToMajor(long id){
        long large_value = Impl::large_value;
        int major = static_cast<int>(id/large_value);
        return major;
    }
    
    
    int Beacon::convertIdToMinor(long id){
        long large_value = Impl::large_value;
        int minor = static_cast<int>(id%large_value);
        return minor;
    }
    
    
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
    
}