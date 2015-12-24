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

namespace loc{
    
    class Beacon;
    class Beacons;

    class Beacon{
        
    private:
        int major_;
        int minor_;
        double rssi_;
        class Impl;
        
    public:
        Beacon(int major, int minor, double rssi);
        ~Beacon();
        
        int major() const;
        int minor() const;
        double rssi() const;
        
        long id() const;
        
        Beacon& major(int major);
        Beacon& minor(int minor);
        Beacon& rssi(double rssi);
       
        std::string toString();
        
        template<class Tbeacons>
        static Tbeacons meanBeaconsVector(std::vector<Tbeacons> beaconsVector);
        
        static long convertMajorMinorToId(int major, int minor);
        static int convertIdToMajor(long id);
        static int convertIdToMinor(long id);
        
        static Beacons filter(const Beacons& beacons, double minRssi, double maxRssi);
        static Beacons sortByRssi(const Beacons& beacons);
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

#endif /* Beacon_hpp */
