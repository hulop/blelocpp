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

#include "MathUtils.hpp"
#include "PosteriorResampler.hpp"

namespace loc{
    
    template<class Tstate>
    void OrientationPosteriorResampler<Tstate>::probabilityParametric(double probParam){
        probParametric = probParam;
    }
    
    template<class Tstate>
    double OrientationPosteriorResampler<Tstate>::probabilityParametric() const{
        return probParametric;
    }
    
    template<class Tstate>
    std::vector<Tstate> OrientationPosteriorResampler<Tstate>::resample(const std::vector<Tstate>& states){
        
        size_t n = states.size();
        States statesNew(n);
        
        std::vector<double> orientations;
        for(size_t i=0; i<n; i++){
            orientations.push_back(states.at(i).orientation());
        }
        
        auto wrappedNormStats = MathUtils::computeWrappedNormalParameters(orientations);
        double mu = wrappedNormStats.mean();
        double sigma = wrappedNormStats.stdev();
        
        for(size_t i=0; i<n; i++){
            State sNew(states.at(i));
            double theta;
            double r = mRand->nextDouble();
            if(isinf(sigma)){
                theta = 2.0*M_PI*mRand->nextDouble();
            }else if(r < probParametric){
                theta = mu + sigma*mRand->nextGaussian();
                theta = std::fmod(theta, 2.0*M_PI);
            }else{
                theta = 2.0*M_PI*mRand->nextDouble();
            }
            theta = theta<M_PI ? theta : theta-2.0*M_PI; // [-pi, pi]
            sNew.orientation(theta);
            sNew.orientationBias(theta);
            statesNew.at(i) = sNew;
        }
        
        return statesNew;
    }
    
    template class OrientationPosteriorResampler<State>;
}
