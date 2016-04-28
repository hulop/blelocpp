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

#include <boost/math/distributions/chi_squared.hpp>
#include "MathUtils.hpp"

double MathUtils::quantileChiSquaredDistribution(int degreeOfFreedom, double cumulativeDensity){
    boost::math::chi_squared chi_sq(degreeOfFreedom);
    double x = boost::math::quantile(chi_sq, cumulativeDensity);
    return x;
}

DirectionalStatistics MathUtils::computeDirectionalStatistics(std::vector<double> orientations){
    size_t n = orientations.size();
    double x = 0, y = 0;
    for(auto ori : orientations){
        x += std::cos(ori);
        y += std::sin(ori);
    }
    x /= n;
    y /= n;
    double meanOri = std::atan2(y,x);
    double R = std::sqrt(x*x + y*y);
    double v = 1.0 - R;
    DirectionalStatistics oristat(meanOri, v);
    return oristat;
}