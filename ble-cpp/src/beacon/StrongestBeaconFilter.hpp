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

#ifndef StrongestBeaconFilter_hpp
#define StrongestBeaconFilter_hpp

#include <stdio.h>
#include "BeaconFilter.hpp"
#include "BLEBeacon.hpp"

namespace loc{
    
    class StrongestBeaconFilter : public BeaconFilter{
    private:
        double cutoffRssi_ = -100;
        int nStrongest_ = 10;
        
        bool adjustsPower_ = false;
        std::map<BeaconId, double> idPowerMap_;
    public:
        StrongestBeaconFilter() = default;
        StrongestBeaconFilter(int nStrongest);
        ~StrongestBeaconFilter() = default;
        
        Beacons filter(const Beacons& beacons) const;
        StrongestBeaconFilter& nStrongest(int nStrongest);
        StrongestBeaconFilter& bleBeacons(const BLEBeacons& bleBeacons);
        StrongestBeaconFilter& adjustsPower(bool adjustsPower);
        StrongestBeaconFilter& cutoffRssi(double cutoffRssi);
    };
}
#endif /* StrongestBeaconFilter_hpp */
