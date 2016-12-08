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

#include "AltitudeManagerSimple.hpp"
#include <valarray>

namespace loc{
    
    void AltitudeManagerSimple::parameters(std::shared_ptr<Parameters> params){
        mParams = params;
    }
    
    void AltitudeManagerSimple::putAltimeter(Altimeter alt){
        
        if(altimeterQueue.size()>=mParams->queueLimit()){
            altimeterQueue.pop_front();
        }
        if(altimeterQueue.size()==0){
            altimeterQueue.push_back(alt);
            return;
        }
        auto last = altimeterQueue.back();
        if(std::abs(last.timestamp() - alt.timestamp()) > mParams->timestampIntervalLimit()){
            std::cout << "timestamp interval between two altimeters is too large. altitudeManager was reset." << std::endl;
            altimeterQueue.clear();
        }
        altimeterQueue.push_back(alt);
    }
    
    double AltitudeManagerSimple::heightChange() const{
        int win = mParams->window();
        if(altimeterQueue.size()<win){
            return 0.0;
        }
        auto n = altimeterQueue.size();
        std::valarray<double> relAlts(win);
        for(int i=0; i<win; i++){
            auto idx = n-1-i;
            auto alt = altimeterQueue.at(idx);
            auto relAlt = alt.relativeAltitude();
            relAlts[i] = relAlt;
        }
        auto mean = relAlts.sum() / relAlts.size();
        auto var = ((relAlts*relAlts).sum() - mean*mean*relAlts.size()) / relAlts.size();
        auto std = std::sqrt(var);
        
        double th = mParams->stdThreshold();
        std::cout << "std_relAlt=" << std << ". (threshold=" << th << ")" << std::endl;
        if(std > th){
            //std::cout << "std_relAlt=" << std << " is gt " << th << "." << std::endl;
            return 1.0;
        }else{
            return 0.0;
        }
    }
}


