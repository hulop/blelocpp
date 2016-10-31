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
#include "LatLngUtil.hpp"

#include <boost/geometry/algorithms/detail/vincenty_direct.hpp>
#include <boost/geometry/util/promote_floating_point.hpp>
#include <boost/geometry/util/select_calculation_type.hpp>
#include <boost/geometry/core/srs.hpp>
#include <boost/geometry/strategies/strategies.hpp>
#include <boost/geometry/algorithms/assign.hpp>
#include <boost/geometry/geometries/point.hpp>

namespace bg = boost::geometry;

namespace loc {
    const double d2r = M_PI / 180.0;
    const double r2d = 180.0 / M_PI;
    
    const double gda_a = 6378.1370;
    const double gda_f = 1.0 / 298.25722210;
    const double gda_b = gda_a * ( 1.0 - gda_f );


    template <typename P>
    bool LatLngUtil::non_precise_ct()
    {
        typedef typename bg::coordinate_type<P>::type ct;
        return boost::is_integral<ct>::value || boost::is_float<ct>::value;
    }
    
    template <typename P, typename Spheroid>
     Point2D LatLngUtil::transform(LatLng latlng, Anchor anchor, Spheroid spheroid) {
        typedef typename bg::promote_floating_point
        <typename bg::select_calculation_type<P, P, void>::type>::type calc_t;
        
        typedef bg::detail::vincenty_inverse<calc_t, true, true> inverse_formula;
        typename inverse_formula::result_type
        result_i = inverse_formula::apply(anchor.latlng.lng * d2r,
                                          anchor.latlng.lat * d2r,
                                          latlng.lng * d2r,
                                          latlng.lat * d2r,
                                          spheroid);
        calc_t dist = result_i.distance;
        calc_t azimuth = ((result_i.azimuth * r2d) - anchor.rotate) * d2r;
        
        Point2D p;
        p.x = sin(azimuth) * dist;
        p.y = cos(azimuth) * dist;
        return p;
    }
    
    template <typename P, typename Spheroid>
     LatLng LatLngUtil::transform(LatLng latlng, double dist, double angle, Spheroid spheroid){
        typedef typename bg::promote_floating_point
        <typename bg::select_calculation_type<P, P, void>::type>::type calc_t;
        
        typedef bg::detail::vincenty_direct<calc_t> direct_formula;
        typename direct_formula::result_type
        result = direct_formula::apply(latlng.lng * d2r,
                                       latlng.lat * d2r,
                                       dist,
                                       angle * d2r,
                                       spheroid);
        
        double lat = result.lat2 * r2d;
        double lng = result.lon2 * r2d;
        LatLng ll;
        ll.lat = lat;
        ll.lng = lng;
        
        return ll;
    }
    

     LatLng LatLngUtil::localToGlobal(const Point2D p, const Anchor anchor) {
        bg::srs::spheroid<double> const gda_spheroid(gda_a, gda_b);
        // usually atan2 takes (y, x) order but rotation is opposite in spherical coordinate
        double r = (atan2(p.x, p.y) * r2d)+anchor.rotate;
        double d = sqrt(pow(p.x,2)+pow(p.y,2)) / 1000; // in kilometer
        
        typedef typename bg::model::point<double, 2, bg::cs::geographic<bg::degree>> point_t;
        return transform<point_t>(anchor.latlng, d, r, gda_spheroid);
    }
    
    Point2D LatLngUtil::globalToLocal(const LatLng latlng, const Anchor anchor) {
        bg::srs::spheroid<double> const gda_spheroid(gda_a, gda_b);
        typedef typename bg::model::point<double, 2, bg::cs::geographic<bg::degree>> point_t;
        Point2D p = transform<point_t>(latlng, anchor, gda_spheroid);
        p.x *= 1000;
        p.y *= 1000;
        return p;
    }
    
};
