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

#ifndef State_hpp
#define State_hpp

#include <stdio.h>
#include <iostream>
#include <vector>
#include "Location.hpp"
#include "Pose.hpp"

namespace loc{
    class State;
    using States = std::vector<State>;
    
    class State : public Pose{
    private:
        double orientationBias_ = 0;
        double rssiBias_ = 0;
        double weight_ = 1.0;
        double negativeLogLikelihood_;
        double mahalanobisDistance_;
        
    public:
        using Ptr = std::shared_ptr<State>;
        
        State() = default;
        ~State() = default;
        
        State(const Pose& pose);
        double orientationBias() const;
        double rssiBias() const;
        double weight() const;
        double negativeLogLikelihood() const;
        double mahalanobisDistance() const;
        State& orientationBias(double orientationBias);
        State& rssiBias(double rssiBias);
        State& weight(double weight);
        State& negativeLogLikelihood(double negativeLogLikelihood);
        State& mahalanobisDistance(double mahalanobisDistance);
        static State mean(const std::vector<State>& states);
        static State weightedMean(const std::vector<State>& states);
        
        // for string stream
        friend std::ostream& operator<<(std::ostream&os, const State& pose);
        
        std::string header() const;
    };
    
    class StateProperty{
        
        double meanRssiBias_ = 0.0; // [dBm/s]
        double stdRssiBias_ = 0.0; // [dBm/s]
        double diffusionRssiBias_ = 0.0; // [dBm/s]
        double diffusionOrientationBias_ = 10.0/180.0*M_PI; // [radian/s]
        
        double minRssiBias_ = -10;
        double maxRssiBias_ = 10;
        
    public:
        using Ptr = std::shared_ptr<StateProperty>;
        
        StateProperty& meanRssiBias(double meanRssiBias);
        double meanRssiBias() const;
        StateProperty& stdRssiBias(double stdRssiBias);
        double stdRssiBias() const;
        StateProperty& diffusionRssiBias(double diffusionRssiBias);
        double diffusionRssiBias() const;
        StateProperty& diffusionOrientationBias(double diffusionOrientationBias);
        double diffusionOrientationBias();
        StateProperty& minRssiBias(double minRssiBias);
        double minRssiBias() const;
        StateProperty& maxRssiBias(double maxRssiBias);
        double maxRssiBias() const;
    };
}
#endif /* State_hpp */
