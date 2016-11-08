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

#ifndef LatLngUtil_hpp
#define LatLngUtil_hpp

#include <stdio.h>
#include <cmath>

namespace loc {
    
    typedef struct {
        double x = 0;
        double y = 0;
    } Point2D;
    
    typedef struct {
        double lat = 0;
        double lng = 0;
    } LatLng;
    
    typedef struct {
        LatLng latlng;
        double rotate = 0; //Degree
    } Anchor;

    class LatLngUtil {
    private:
        template <typename P>
        bool non_precise_ct();
        template <typename P, typename Spheroid>
        static Point2D transform(LatLng latlng, Anchor anchor, Spheroid spheroid);
        
        template <typename P, typename Spheroid>
        static LatLng transform(LatLng latlng, double dist, double angle, Spheroid spheroid);
        
    public:
        static LatLng localToGlobal(const Point2D p, const Anchor anchor);
        static Point2D globalToLocal(const LatLng latlng, const Anchor anchor);
    };
    
}

#endif /* LatLngUtil_hpp */
