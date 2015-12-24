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

#include "Acceleration.hpp"

namespace loc{
    
    Acceleration::Acceleration(long timestamp, double ax, double ay, double az){
        timestamp_ = timestamp;
        ax_ = ax;
        ay_ = ay;
        az_ = az;
    }
    
    Acceleration::~Acceleration(){}
    
    Acceleration* Acceleration::ax(double ax){
        this->ax_ = ax;
        return this;
    }
    
    Acceleration* Acceleration::ay(double ay){
        this->ay_ = ay;
        return this;
    }
    
    Acceleration* Acceleration::az(double az){
        this->az_ = az;
        return this;
    }
    
    long Acceleration::timestamp() const{
        return timestamp_;
    }
    
    double Acceleration::ax() const{
        return ax_;
    }
    
    double Acceleration::ay() const{
        return ay_;
    }
    
    double Acceleration::az() const{
        return az_;
    }
}

