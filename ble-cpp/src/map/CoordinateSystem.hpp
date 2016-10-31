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

#ifndef CoordinateSystem_hpp
#define CoordinateSystem_hpp


#define COORDINATE_SYSTEM_2D

#include <stdio.h>
#include "LocException.hpp"

namespace loc{
    // Parameters to define coordinate transformation
    struct CoordinateSystemParameters{
        double punit_x = 1;
        double punit_y = 1;
        double punit_z = 1;
        double x_origin = 0;
        double y_origin = 0;
        double z_origin = 0;
        
        CoordinateSystemParameters() = default;
        ~CoordinateSystemParameters() = default;
        
        CoordinateSystemParameters(double punit_x, double punit_y, double punit_z, double x_origin, double y_origin, double z_origin);
    };

    
    class CoordinateSystem {
        CoordinateSystemParameters mPara;
    public:
        CoordinateSystem(){}
        CoordinateSystem(CoordinateSystemParameters parameters){
            mPara = parameters;
        }
        template<class Tstate> Tstate worldToLocalState(const Tstate& state) const;
        template<class Tstate> Tstate localToWorldState(const Tstate& state) const;
    };
    
    template<class Tstate>
    Tstate CoordinateSystem::worldToLocalState(const Tstate& state) const{
        double x_local = mPara.punit_x * state.x() + mPara.x_origin;
        double y_local = mPara.punit_y * state.y() + mPara.y_origin;
        
        Tstate stateNew(state);
        stateNew.x(x_local);
        stateNew.y(y_local);
#ifndef COORDINATE_SYSTEM_2D
        double z_local = mPara.punit_z * state.z() + mPara.z_origin;
        stateNew.z(z_local);
#endif
        return stateNew;
    }
    
    template<class Tstate>
    Tstate CoordinateSystem::localToWorldState(const Tstate& locCoord) const{
        double x_world = (locCoord.x() - mPara.x_origin)/mPara.punit_x;
        double y_world = (locCoord.y() - mPara.y_origin)/mPara.punit_y;
        
        Tstate stateNew(locCoord);
        stateNew.x(x_world);
        stateNew.y(y_world);
#ifndef COORDINATE_SYSTEM_2D
        double z_world = (locCoord.z() - mPara.z_origin)/mPara.punit_z;
        stateNew.z(z_world);
#endif
        return stateNew;
    }
}
#endif /* CoordinateSystem_hpp */
