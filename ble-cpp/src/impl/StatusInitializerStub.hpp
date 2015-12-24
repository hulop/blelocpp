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

#ifndef StatusInitializerStub_hpp
#define StatusInitializerStub_hpp

#include <stdio.h>
#include "StatusInitializer.hpp"

namespace loc{
    
    class StatusInitializerStub : public StatusInitializer{
    public:
        Locations initializeLocations(int n){
            Locations locs;
            //int n= 100;
            for(int i=0; i<n; i++){
                Location loc(0,0,0,0);
                locs.push_back(loc);
            }
            return locs;
        }
        
        Poses initializePoses(int n){
            Locations locs = initializeLocations(n);
            
            //int n = (int) locs.size();
            
            Poses poses(n);
            for(int i=0; i<n; i++){
                Location loc = locs.at(i);
                Pose pose;
                pose.x(loc.x()).y(loc.y()).floor(loc.floor()).z(loc.z());
                pose.orientation(0.0).velocity(1.0).normalVelocity(1.0);
                poses[i]=pose;
            }
            return poses;
        }
        States initializeStates(int n){
            Poses poses = initializePoses(n);
            //int n = (int) poses.size();
            States states(n);
            
            for(int i=0; i<n; i++){
                Pose pose = poses.at(i);
                State state;
                state.x(pose.x()).y(pose.y()).floor(pose.floor()).z(pose.z());
                state.orientation(0.0).velocity(1.0).normalVelocity(1.0);
                state.orientationBias(0.0).rssiBias(0.0);
                states[i]=state;
            }
            return states;
        }
    };
}

#endif /* StatusInitializerStub_hpp */
