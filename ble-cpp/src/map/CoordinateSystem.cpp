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

#include "CoordinateSystem.hpp"

namespace loc{
    CoordinateSystemParameters::CoordinateSystemParameters(double punit_x, double punit_y, double punit_z, double x_origin, double y_origin, double z_origin){
        this->punit_x = punit_x;
        this->punit_y = punit_y;
        this->punit_z = punit_z;
        this->x_origin = x_origin;
        this->y_origin = y_origin;
        this->z_origin = z_origin;
        
        if(this->punit_x == 0){
            BOOST_THROW_EXCEPTION(LocException("punit_x == 0"));
        }
        if(this->punit_y == 0){
            BOOST_THROW_EXCEPTION(LocException("punit_x == 0"));
        }
        
    }

}
