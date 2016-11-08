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
#ifndef Status_hpp
#define Status_hpp

#include <iostream>
#include <vector>
#include <memory>


#include "Location.hpp"
#include "Pose.hpp"
#include "State.hpp"
//#include "State.hpp"

namespace loc{
    
    class Status{
        
    public:
        Status();
        ~Status();
        
        enum Step{
            PREDICTION,
            RESET,
            FILTERING_WITH_RESAMPLING,
            FILTERING_WITHOUT_RESAMPLING,
            OBSERVATION_WITHOUT_FILTERING,
            OTHER
        };
        
        std::shared_ptr<Location> meanLocation() const;
        std::shared_ptr<Pose> meanPose() const;
        long timestamp() const;
        std::shared_ptr<std::vector<State>> states() const;
        Step step() const;
        
        Status& meanLocation(Location* loc);
        Status& meanLocation(std::shared_ptr<Location> location);
        Status& meanPose(Pose* pose);
        Status& meanPose(std::shared_ptr<Pose> pose);
        Status& timestamp(long timestamp);
        Status& states(std::vector<State>* states);
        Status& step(Step step);
        
    private:
        long timestamp_;
        Step step_ = Step::OTHER;
        std::shared_ptr<Location> meanLocation_;
        std::shared_ptr<Pose> meanPose_;
        std::shared_ptr<std::vector<State>> states_;
    };
    
}

#endif /* Status_hpp */
