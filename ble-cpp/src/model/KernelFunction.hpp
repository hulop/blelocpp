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

#ifndef KernelFunction_hpp
#define KernelFunction_hpp

#include <stdio.h>
#include <iostream>
#include <cmath>


class KernelFunction{
public:
    virtual ~KernelFunction(){}
    virtual double computeKernel(const double x1[], const double x2[]) const = 0;
    virtual double variance() const = 0;
    
};

class GaussianKernel : public KernelFunction{
private:
    static const int ndim = 4;
    double variance_ = 1.0*1.0;
    
public:
    struct Parameters{
        double sigma_f = 1.0;
        double lengthes[ndim] = {2.0, 2.0, 2.0, 0.01};
        
        template<class Archive>
        void serialize(Archive& ar);
        std::string toString() const;
    };
    
    GaussianKernel() = default;
    ~GaussianKernel() = default;
    GaussianKernel(Parameters params);
    
    double computeKernel(const double x1[], const double x2[]) const override;
    double variance() const override;
    double sqsum(const double x1[], const double x2[]) const;
    
    template<class Archive>
    void save(Archive& ar) const;
    template<class Archive>
    void load(Archive& ar);
    
private:
    Parameters params;
    
};



#endif /* KernelFunction_hpp */
