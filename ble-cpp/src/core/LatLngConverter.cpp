/*******************************************************************************
 * Copyright (c) 2014, 2016  IBM Corporation and others
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

#include "LatLngConverter.hpp"
#include "BLEBeacon.hpp"
#include "State.hpp"

namespace loc{
    
    LatLngConverter::LatLngConverter(const Anchor& anchor){
        this->anchor(anchor);
    }
    
    void LatLngConverter::anchor(const Anchor& anchor){
        anchor_ = anchor;
    };
    Anchor LatLngConverter::anchor() const{
        return anchor_;
    }
    
    template<class Tstate>
    GlobalState<Tstate> LatLngConverter::localToGlobal(const Tstate& local){
        Point2D p;
        p.x = local.x();
        p.y = local.y();
        auto latlng = LatLngUtil::localToGlobal(p, anchor_);
        GlobalState<Tstate> gs(local);
        gs.lat(latlng.lat);
        gs.lng(latlng.lng);
        return gs;
    };
    
    template<class Tstate>
    Tstate LatLngConverter::globalToLocal(const GlobalState<Tstate>& global){
        LatLng latlng;
        latlng.lat = global.lat();
        latlng.lng = global.lng();
        auto p = LatLngUtil::globalToLocal(latlng, anchor_);
        Tstate s(global);
        s.x(p.x);
        s.y(p.y);
        return s;
    }
    
    template GlobalState<Location> LatLngConverter::localToGlobal<Location>(const Location& local);
    template Location LatLngConverter::globalToLocal<Location>(const GlobalState<Location>& global);
    template GlobalState<BLEBeacon> LatLngConverter::localToGlobal<BLEBeacon>(const BLEBeacon& local);
    template BLEBeacon LatLngConverter::globalToLocal<BLEBeacon>(const GlobalState<BLEBeacon>& global);
    template GlobalState<Pose> LatLngConverter::localToGlobal<Pose>(const Pose& local);
    template Pose LatLngConverter::globalToLocal<Pose>(const GlobalState<Pose>& global);
    template GlobalState<State> LatLngConverter::localToGlobal<State>(const State& local);
    template State LatLngConverter::globalToLocal<State>(const GlobalState<State>& global);
    
    
    double LatLngConverter::headingGlobalToLocal(double heading){
        double localHeading = ( heading - anchor_.rotate )/180.0*M_PI; // radian
        double xH = std::sin(localHeading);
        double yH = std::cos(localHeading);
        double orientation = atan2(yH,xH);
        return orientation;
    }
    
    LocalHeading LatLngConverter::headingGlobalToLocal(const Heading& heading){
        long ts = heading.timestamp();
        double trueHeading = heading.trueHeading(); // deg
        double headingAcc = heading.headingAccuracy(); // deg
        
        if(trueHeading < 0){ // if trueHeading == -1, the value is invalid
            // update trueHeading value here immediately before creating locHead
            if(!std::isnan(anchor_.magneticDeclination)){
                trueHeading = heading.magneticHeading() + anchor_.magneticDeclination;
            }else{
                std::cerr << "trueHeading is uncertain. magneticHeading is used instead." << std::endl;
                trueHeading = heading.magneticHeading();
            }
        }

        double ori = this->headingGlobalToLocal(trueHeading);
        double oriDev = headingAcc/180.0*M_PI;
        
        LocalHeading locHead(ts, ori, oriDev);
        return locHead;
    }
}
