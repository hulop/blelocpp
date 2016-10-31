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

#include <stdio.h>

#include "State.hpp"


namespace  loc {
    
    State::State(const Pose& pose): Pose(pose){}
    
    double State::orientationBias() const{
        return orientationBias_;
    }
    
    double State::rssiBias() const{
        return rssiBias_;
    }
    
    double State::weight() const{
        return weight_;
    }
    
    double State::negativeLogLikelihood() const{
        return negativeLogLikelihood_;
    }
    
    double State::mahalanobisDistance() const{
        return mahalanobisDistance_;
    }
    
    State& State::orientationBias(double orientationBias){
        orientationBias_ = orientationBias;
        return *this;
    }
    
    State& State::rssiBias(double rssiBias){
        rssiBias_ = rssiBias;
        return *this;
    }
    
    State& State::weight(double weight){
        weight_ = weight;
        return *this;
    }
    
    State& State::negativeLogLikelihood(double negativeLogLikelihood){
        negativeLogLikelihood_ = negativeLogLikelihood;
        return *this;
    }
    
    State& State::mahalanobisDistance(double mahalanobisDistance){
        mahalanobisDistance_ = mahalanobisDistance;
        return *this;
    }
    
    // for string stream
    std::ostream& operator<<(std::ostream&os, const State& state){
        os << state.x() <<"," << state.y() <<"," << state.z() <<"," << state.floor()
        << "," << state.orientation() << "," << state.velocity() <<"," << state.normalVelocity()
        << "," << state.orientationBias() << "," << state.rssiBias() << "," << state.weight()
        << "," << state.negativeLogLikelihood() << "," << state.mahalanobisDistance();
        return os;
    }
    
    std::string State::header() const{
        std::string str;
        str = "x,y,z,floor,orientation,velocity,normalVelocity,orientationBias,rssiBias,weight,negativeLogLikelihood,mahalanobisDistance";
        return str;
    }
    
    
    State State::mean(const std::vector<State>& states){
        Pose meanPose = Pose::mean<State>(states);
        State meanState(meanPose);
        
        double meanRssiBias = 0;
        double xOri = 0;
        double yOri = 0;
        for(const State& s: states){
            meanRssiBias += s.rssiBias();
            xOri += std::cos(s.orientationBias());
            yOri += std::sin(s.orientationBias());
        }
        meanRssiBias/=(double)states.size();
        double meanOriB = std::atan2(yOri, xOri);
        meanState.rssiBias(meanRssiBias);
        meanState.orientationBias(meanOriB);
        return meanState;
    }
    
    State State::weightedMean(const std::vector<State>& states){
        size_t n = states.size();
        std::vector<double> weights(n);
        double weightSum = 0;
        for(int i=0; i<n; i++){
            weights[i] = states.at(i).weight();
            weightSum += weights[i];
        }
        
        Pose meanPose = Pose::weightedMean<State>(states, weights);
        State meanState(meanPose);
        
        double meanRssiBias = 0;
        double xOri = 0;
        double yOri = 0;
        for(const State& s: states){
            double w = s.weight() / weightSum;
            meanRssiBias += s.rssiBias()*w;
            xOri += std::cos(s.orientationBias()*w);
            yOri += std::sin(s.orientationBias()*w);
        }
        //meanRssiBias/=(double)states.size();
        double meanOriB = std::atan2(yOri, xOri);
        meanState.rssiBias(meanRssiBias);
        meanState.orientationBias(meanOriB);
        return meanState;
    }
    
    
    // State Property
    StateProperty& StateProperty::meanRssiBias(double meanRssiBias){
        meanRssiBias_ = meanRssiBias;
        return *this;
    }
    
    double StateProperty::meanRssiBias() const{
        return meanRssiBias_;
    }
    
    StateProperty& StateProperty::stdRssiBias(double stdRssiBias){
        stdRssiBias_ = stdRssiBias;
        return *this;
    }
    
    double StateProperty::stdRssiBias() const{
        return stdRssiBias_;
    }
    
    StateProperty& StateProperty::diffusionRssiBias(double diffusionRssiBias){
        diffusionRssiBias_ = diffusionRssiBias;
        return *this;
    }
    
    double StateProperty::diffusionRssiBias() const{
        return diffusionRssiBias_;
    }
    
    StateProperty& StateProperty::diffusionOrientationBias(double diffusionOrientationBias){
        diffusionOrientationBias_ = diffusionOrientationBias;
        return *this;
    }
    
    double StateProperty::diffusionOrientationBias(){
        return diffusionOrientationBias_;
    }
    
    StateProperty& StateProperty::minRssiBias(double minRssiBias){
        minRssiBias_ = minRssiBias;
        return *this;
    }
    
    double StateProperty::minRssiBias() const{
        return minRssiBias_;
    }
    
    StateProperty& StateProperty::maxRssiBias(double maxRssiBias){
        maxRssiBias_ = maxRssiBias;
        return *this;
    }
    
    double StateProperty::maxRssiBias() const{
        return maxRssiBias_;
    }
    
}