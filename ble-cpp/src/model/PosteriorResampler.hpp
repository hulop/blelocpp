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

#ifndef PosteriorResampler_hpp
#define PosteriorResampler_hpp

#include <stdio.h>
#include <memory>
#include "RandomGenerator.hpp"
#include "State.hpp"

namespace loc {
    
    template<class Tstate>
    class PosteriorResampler{
    public:
        using Ptr = std::shared_ptr<PosteriorResampler<Tstate>>;
        
        virtual ~PosteriorResampler() = default;
        virtual std::vector<Tstate> resample(const std::vector<Tstate>& states) = 0;
    };
    
    template<class Tstate>
    class OrientationPosteriorResampler : public PosteriorResampler<Tstate>{
        
    private:
        std::shared_ptr<RandomGenerator> mRand;
        double probParametric = 0.8;
        
    public:
        using Ptr = std::shared_ptr<OrientationPosteriorResampler<Tstate>>;
        
        OrientationPosteriorResampler(): mRand(new RandomGenerator){}
        ~OrientationPosteriorResampler() = default;
        
        std::vector<Tstate> resample(const std::vector<Tstate>&);
        void probabilityParametric(double);
        double probabilityParametric() const;
    };
}



#endif /* PosteriorResampler_hpp */
