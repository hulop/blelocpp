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

#include "GridResampler.hpp"

namespace loc{
    
    class Location;
    class Pose;
    
    template<class Tstate> std::vector<Tstate>* GridResampler<Tstate>::resample(const std::vector<Tstate>& states, const double weights[]){
        
        int n = (int) states.size();
        std::vector<Tstate>* statesResampled = new std::vector<Tstate>();
        
        double d = rand.nextDouble();
        std::vector<double> grid(n);
        
        for (int k=0; k<n; k++){
            if(gtype==STRATIFIED){
                d = rand.nextDouble();
            }
            grid[k] = ((double)k + d)/((double)n);
        }
        
        double cumWeight=0;
        int k=0;
        for(int i=0; i<n; i++){
            int numSample = 0;
            cumWeight += weights[i];
            if(i==n-1){
                cumWeight = 1.0;
            }
            for( ; k<n; k++){
                if(grid[k] < cumWeight){
                    Tstate stateCopied(states.at(i));
                    statesResampled->push_back(stateCopied);
                    numSample++;
                }else{
                    break;
                }
            }
        }
        return statesResampled;
    }
    
    // Explicit instantiation
    template class GridResampler<Location>;
    template class GridResampler<Pose>;
    template class GridResampler<State>;
    
}