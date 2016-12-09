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
#include <sstream>
#include <valarray>

namespace loc{
    
    void AltitudeManagerSimple::parameters(std::shared_ptr<Parameters> params){
        mParams = params;
    }
    
    void AltitudeManagerSimple::putAltimeter(Altimeter alt){
        
        std::lock_guard<std::mutex> lock(mtx_);
        
        if(altimeterQueue.size()>=mParams->queueLimit()){
            altimeterQueue.pop_front();
        }
        if(altimeterQueue.size()==0){
            altimeterQueue.push_back(alt);
            return;
        }
        auto last = altimeterQueue.back();
        if(std::abs(last.timestamp() - alt.timestamp()) > mParams->timestampIntervalLimit()){
            if(verbose()){
                std::cout << "timestamp interval between two altimeters is too large. altitudeManager was reset." << std::endl;
            }
            altimeterQueue.clear();
        }
        altimeterQueue.push_back(alt);
    }
    
    void AltitudeManagerSimple::verbose(bool verbose){
        verbose_ = verbose;
    }
    
    bool AltitudeManagerSimple::verbose() const{
        return verbose_;
    }
    
    double AltitudeManagerSimple::heightChange() const{
        int win = mParams->window();
        std::deque<Altimeter> queueTmp;
        queueTmp = altimeterQueue;

        if(queueTmp.size()<win){
            return 0.0;
        }
        auto n = queueTmp.size();
        std::valarray<double> relAlts(win);
        for(int i=0; i<win; i++){
            auto idx = n-1-i;
            auto alt = queueTmp.at(idx);
            auto relAlt = alt.relativeAltitude();
            relAlts[i] = relAlt;
        }
        double mean = relAlts.sum()/relAlts.size();
        double var = ((relAlts*relAlts).sum() - mean*mean*relAlts.size())/relAlts.size();
        var = std::max(0.0, var);
        double stdev = std::sqrt(var);
        
        double th = mParams->stdThreshold();
        std::stringstream ss;
        ss << "std_relAlt=" << stdev << ". (threshold=" << th << ")";
        double retVal = 0.0;
        if(stdev > th){
            ss << " Height change detected.";
            retVal =  1.0;
        }
        if(verbose()){
            std::cout << ss.str() << std::endl;
        }
        return retVal;
    }
}


