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

#ifndef SystemModel_hpp
#define SystemModel_hpp

#include <stdio.h>
#include <vector>
#include <memory>

#include "Location.hpp"

namespace loc{
    
class SystemModelInput{
    long timestamp_;
    long previousTimestamp_;
    static constexpr double timeUnit_ = 0.001; // ms to s
        
public:
    void timestamp(long timestamp){
        timestamp_ = timestamp;
    }
        
    void previousTimestamp(long previousTimestamp){
        previousTimestamp_ = previousTimestamp;
    }
        
    long timestamp() const{
        return timestamp_;
    }
    long previousTimestamp() const{
        return previousTimestamp_;
    }
    double timeUnit() const{
        return timeUnit_;
    }
};
    
    template<class Ts, class Tin> class SystemModel{
    public:
        
        using Ptr = std::shared_ptr<SystemModel<Ts, Tin>>;
        
        virtual ~SystemModel(){}
        
        //virtual SystemModel<Ts, Tin, Tproperty>* setProperty(Tproperty property) = 0;
        virtual Ts predict(Ts state, Tin input) = 0;
        virtual std::vector<Ts> predict(std::vector<Ts> states, Tin input)  = 0;
        //virtual std::vector<Ts>* predict(std::vector<Ts> states) = 0;
        
        virtual void startPredictions(const std::vector<Ts>& states, const Tin& input){
            // Do nothing in a default method
        }
        virtual void endPredictions(const std::vector<Ts>& states, const Tin& input){
            // Do nothing in a default method
        }
        virtual void notifyObservationUpdated(){
            // Do nothing in a default method
        }
    }; 
    /*
     template class SystemModel<Location, Input>;
     */
    
    class SystemModelVelocityAdjustable{
    protected:
        double velocityRate_ = 1.0;
        double relativeVelocity_ = 0.0;
    public:
        using Ptr = std::shared_ptr<SystemModelVelocityAdjustable>;
        virtual ~SystemModelVelocityAdjustable(){}
        virtual void velocityRate(double velocityRate);
        virtual double velocityRate() const;
        virtual void relativeVelocity(double relVel);
        virtual double relativeVelocity() const;
    };
    
    class SystemModelMovementControllable{
    protected:
        bool isUnderControll = false;
        double mMovement = 0;
    public:
        virtual ~SystemModelMovementControllable() = default;
        void forceMove();
        void forceStop();
        void controlMovement(double movement);
        void releaseControl();
    };
}

#endif /* SystemModel_hpp */
