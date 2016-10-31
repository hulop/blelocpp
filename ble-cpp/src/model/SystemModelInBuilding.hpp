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

#ifndef SystemModelInBuilding_hpp
#define SystemModelInBuilding_hpp

#include <stdio.h>
#include "RandomWalker.hpp"
#include "RandomWalkerMotion.hpp"
#include "PoseRandomWalkerInBuilding.hpp"
#include "Building.hpp"

namespace loc{
    
    class SystemModelInBuildingProperty: public PoseRandomWalkerInBuildingProperty{
    public:
        using Ptr = std::shared_ptr<SystemModelInBuildingProperty>;
        
        SystemModelInBuildingProperty(const PoseRandomWalkerInBuildingProperty& property):
        PoseRandomWalkerInBuildingProperty(property){}
        
    private:
        double probabilityJump_ = 0.0;
        
    public:
        void probabilityJump(double probability){
            probabilityJump_ = probability;
        }

        double probabilityJump() const{
            return probabilityJump_;
        }
    };
    
    template<class Tstate, class Tinput>
    class SystemModelInBuilding: public SystemModel<Tstate, Tinput>{
        
    public:
        using SystemModelT = SystemModel<Tstate, Tinput>;
        using Ptr = std::shared_ptr<SystemModelInBuilding>;
        
    private:
        RandomGenerator mRandomGenerator;
        typename SystemModel<Tstate, Tinput>::Ptr mSysModel;
        Building::Ptr mBuilding;
        SystemModelInBuildingProperty::Ptr mProperty;
        
        Tstate moveOnElevator(const Tstate& state, Tinput input);
        Tstate moveOnStair(const Tstate& state, Tinput input);
        Tstate moveOnEscalator(const Tstate& state, Tinput input);
        Tstate moveOnFloor(const Tstate& state, Tinput input);
        Tstate moveOnFloorRetry(const Tstate& state, const Tstate& stateNew,  Tinput input);
        Tstate moveFloorJump(const Tstate& state, Tinput input);
        
    public:
        
        SystemModelInBuilding() = default;
        virtual ~SystemModelInBuilding() = default;
        
        SystemModelInBuilding(typename SystemModelT::Ptr poseRandomWalker, Building::Ptr building, SystemModelInBuildingProperty::Ptr property);
        
        SystemModelInBuilding& systemModel(typename SystemModelT::Ptr sysModel);
        SystemModelInBuilding& building(Building::Ptr building);
        SystemModelInBuilding& property(SystemModelInBuildingProperty::Ptr property);
        
        Tstate predict(Tstate state, Tinput input) override;
        std::vector<Tstate> predict(std::vector<Tstate> states, Tinput input) override;
        
        virtual void notifyObservationUpdated() override;
        
    };
    
    // This class is retained for compatibility.
    class PoseRandomWalkerInBuilding: public SystemModelInBuilding<State, SystemModelInput>{
        
    public:
        using Ptr = std::shared_ptr<PoseRandomWalkerInBuilding>;
        
        virtual ~PoseRandomWalkerInBuilding() = default;
        
        virtual void poseRandomWalkerInBuildingProperty(PoseRandomWalkerInBuildingProperty::Ptr property){
            SystemModelInBuildingProperty::Ptr sProperty(new SystemModelInBuildingProperty(*property));
            SystemModelInBuilding<State, SystemModelInput>::property(sProperty);
        }
        
        virtual void poseRandomWalker(typename SystemModel<State, SystemModelInput>::Ptr sysModel){
            SystemModelInBuilding<State, SystemModelInput>::systemModel(sysModel);
        }
        
    };
}


#endif /* SystemModelInBuilding_hpp */
