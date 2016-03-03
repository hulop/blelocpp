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


#include "Attitude.hpp"

namespace loc{
    
    Attitude::Attitude(long timestamp, double pitch, double roll, double yaw){
        timestamp_ = timestamp;
        pitch_ = pitch;
        roll_ = roll;
        yaw_ = yaw;
    }
    
    Attitude::~Attitude(){}
    
    Attitude* Attitude::pitch(double pitch){
        this->pitch_ = pitch;
        return this;
    }
    
    Attitude* Attitude::roll(double roll){
        this->roll_ = roll;
        return this;
    }
    
    Attitude* Attitude::yaw(double yaw){
        this->yaw_ = yaw;
        return this;
    }
    
    long Attitude::timestamp() const{
        return timestamp_;
    }
    
    double Attitude::pitch() const{
        return pitch_;
    }
    
    double Attitude::roll() const{
        return roll_;
    }
    
    double Attitude::yaw() const{
        return yaw_;
    }
    
    std::ostream& operator<<(std::ostream&os, const loc::Attitude& att){
        return os<<att.roll() << "," << att.pitch() << "," << att.yaw();
    }
}