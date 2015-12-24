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

#include "RandomWalker.hpp"
#include <random>

namespace loc{
    
    template<class Ts, class Tin>
    RandomWalker<Ts, Tin>& RandomWalker<Ts, Tin>::setProperty(RandomWalkerProperty property){
        mRWProperty = property;
        return *this;
    }
    
    template<class Ts, class Tin>
    Ts RandomWalker<Ts, Tin>::predict(const Ts loc, const Tin input){
        
        double x = loc.x();
        double y = loc.y();
        double z = loc.z();
        double floor = loc.floor();
        
        x += mRWProperty.sigma * mRandGen.nextGaussian();
        y += mRWProperty.sigma * mRandGen.nextGaussian();
        
        Ts locNew(x, y, z, floor);
        return locNew;
    }
    
    template<class Ts, class Tin>
    std::vector<Ts> RandomWalker<Ts, Tin>::predict(std::vector<Ts> locations, Tin input){
        std::vector<Ts> locsNew;
        
        for(Ts loc: locations){
            Location locNew = predict(loc, input);
            locsNew.push_back(locNew);
        }
        
        return locsNew;
    }
}
