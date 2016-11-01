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

#include "SystemModelInBuilding.hpp"

namespace loc{
    
    // Property
    SystemModelInBuildingProperty& SystemModelInBuildingProperty::probabilityUp(double probabilityUp){
        probabilityUp_ = probabilityUp;
        return *this;
    }
    
    SystemModelInBuildingProperty& SystemModelInBuildingProperty::probabilityDown(double probabilityDown){
        probabilityDown_ = probabilityDown;
        return *this;
    }
    
    SystemModelInBuildingProperty& SystemModelInBuildingProperty::probabilityStay(double probabilityStay){
        probabilityStay_ = probabilityStay;
        return *this;
    }
    
    SystemModelInBuildingProperty& SystemModelInBuildingProperty::wallCrossingAliveRate(double wallCrossingAliveRate){
        wallCrossingAliveRate_ = wallCrossingAliveRate;
        return *this;
    }
    
    SystemModelInBuildingProperty& SystemModelInBuildingProperty::maxIncidenceAngle(double maxIncidenceAngle){
        maxIncidenceAngle_ = maxIncidenceAngle;
        return *this;
    }
    
    SystemModelInBuildingProperty& SystemModelInBuildingProperty::velocityRateFloor(double velocityRateFloor){
        velocityRateFloor_ = velocityRateFloor;
        return *this;
    }
    
    SystemModelInBuildingProperty& SystemModelInBuildingProperty::velocityRateStair(double velocityRateStair){
        velocityRateStair_ = velocityRateStair;
        return *this;
    }
    
    SystemModelInBuildingProperty& SystemModelInBuildingProperty::velocityRateElevator(double velocityRateElevator){
        velocityRateElevator_ = velocityRateElevator;
        return *this;
    }
    
    SystemModelInBuildingProperty& SystemModelInBuildingProperty::velocityRateEscalator(double velocityRateEscalator){
        velocityRateEscalator_ = velocityRateEscalator;
        return *this;
    }
    
    SystemModelInBuildingProperty& SystemModelInBuildingProperty::weightDecayRate(double weightDecayRate){
        weightDecayRate_ = weightDecayRate;
        return *this;
    }
    
    SystemModelInBuildingProperty& SystemModelInBuildingProperty::probabilityFloorJump(double probabilityFloorJump){
        probabilityFloorJump_ = probabilityFloorJump;
        return *this;
    }
    
    double SystemModelInBuildingProperty::probabilityUp() const{
        return probabilityUp_;
    }
    
    double SystemModelInBuildingProperty::probabilityDown() const{
        return probabilityDown_;
    }
    
    double SystemModelInBuildingProperty::probabilityStay() const{
        return probabilityStay_;
    }
    
    double SystemModelInBuildingProperty::wallCrossingAliveRate() const{
        return wallCrossingAliveRate_;
    }
    
    double SystemModelInBuildingProperty::maxIncidenceAngle() const{
        return maxIncidenceAngle_;
    }
    
    double SystemModelInBuildingProperty::velocityRateFloor() const{
        return velocityRateFloor_;
    }
    
    double SystemModelInBuildingProperty::velocityRateStair() const{
        return velocityRateStair_;
    }
    
    double SystemModelInBuildingProperty::velocityRateElevator() const{
        return velocityRateElevator_;
    }
    
    double SystemModelInBuildingProperty::velocityRateEscalator() const{
        return velocityRateEscalator_;
    }
    
    double SystemModelInBuildingProperty::weightDecayRate() const{
        return weightDecayRate_;
    }
    
    int SystemModelInBuildingProperty::maxTrial() const{
        return maxTrial_;
    }
    
    double SystemModelInBuildingProperty::probabilityFloorJump() const{
        return probabilityFloorJump_;
    }
    
    
    // Function definitions
    
    template<class Tstate, class Tinput>
    SystemModelInBuilding<Tstate, Tinput>& SystemModelInBuilding<Tstate, Tinput>::systemModel(typename SystemModelT::Ptr sysModel){
        mSysModel = sysModel;
        return *this;
    }
    
    template<class Tstate, class Tinput>
    SystemModelInBuilding<Tstate, Tinput>::SystemModelInBuilding(typename SystemModelT::Ptr sysModel, Building::Ptr building, SystemModelInBuildingProperty::Ptr property){
        mSysModel = sysModel;
        mBuilding = building;
        mProperty = property;
    }
        
    template<class Tstate, class Tinput>
    SystemModelInBuilding<Tstate, Tinput>& SystemModelInBuilding<Tstate, Tinput>::building(Building::Ptr building){
        mBuilding = building;
        return *this;
    }
    
    template<class Tstate, class Tinput>
    SystemModelInBuilding<Tstate, Tinput>& SystemModelInBuilding<Tstate, Tinput>::property(SystemModelInBuildingProperty::Ptr property){
        mProperty = property;
        return *this;
    }
    
    
    
    template<class Tstate, class Tinput>
    Tstate SystemModelInBuilding<Tstate, Tinput>::moveOnElevator(const Tstate& state, Tinput input){
        int f_min = mBuilding->minFloor();
        int f_max = mBuilding->maxFloor();
        Tstate stateNew;
        while(true){
            int f_new = f_min + mRandomGenerator.nextInt(f_max - f_min);
            if(mBuilding->isValidFloor(f_new)){
                stateNew = Tstate(state);
                stateNew.floor(f_new);
                if(mBuilding->isMovable(stateNew) && mBuilding->isElevator(stateNew)){
                    break;
                }
            }
        }
        return stateNew;
    }
    
    
    template<class Tstate, class Tinput>
    Tstate SystemModelInBuilding<Tstate, Tinput>::moveOnEscalator(const Tstate& state, Tinput input){
        // TODO: many duplications with moveOnStair
        int f_min = mBuilding->minFloor();
        int f_max = mBuilding->maxFloor();
        int f = state.floor();
        int f_new;
        
        double pUp = mProperty->probabilityUp();
        double pDown = mProperty->probabilityDown();
        double pStay = mProperty->probabilityStay();
        
        if(f==f_min){
            pDown = 0;
            pUp = mProperty->probabilityDown()/(mProperty->probabilityDown() + mProperty->probabilityStay());
            pStay = 1 - pUp;
        }else if(f==f_max){
            pDown = mProperty->probabilityUp()/(mProperty->probabilityUp() + mProperty->probabilityStay());
            pUp = 0;
            pStay = 1 - pDown;
        }
        
        Tstate stateNew(state);
        while(true){
            double p = mRandomGenerator.nextDouble();
            if(p < pUp){
                f_new = f+1;
            }else if( p - pUp < pDown){
                f_new = f-1;
            }else{
                f_new = f;
            }
            if(mBuilding->isValidFloor(f_new)){
                stateNew = Tstate(state);
                stateNew.floor(f_new);
                if(mBuilding->isMovable(stateNew) && mBuilding->isEscalator(stateNew)){
                    break;
                }
            }
        }
        return stateNew;
    }
    
    
    template<class Tstate, class Tinput>
    Tstate SystemModelInBuilding<Tstate, Tinput>::moveOnStair(const Tstate& state, Tinput input){
        int f_min = mBuilding->minFloor();
        int f_max = mBuilding->maxFloor();
        int f = state.floor();
        int f_new;
        
        double pUp = mProperty->probabilityUp();
        double pDown = mProperty->probabilityDown();
        double pStay = mProperty->probabilityStay();
        
        if(f==f_min){
            pDown = 0;
            pUp = mProperty->probabilityDown()/(mProperty->probabilityDown() + mProperty->probabilityStay());
            pStay = 1 - pUp;
        }else if(f==f_max){
            pDown = mProperty->probabilityUp()/(mProperty->probabilityUp() + mProperty->probabilityStay());
            pUp = 0;
            pStay = 1 - pDown;
        }
        
        Tstate stateNew(state);
        while(true){
            double p = mRandomGenerator.nextDouble();
            if(p < pUp){
                f_new = f+1;
            }else if( p - pUp < pDown){
                f_new = f-1;
            }else{
                f_new = f;
            }
            if(mBuilding->isValidFloor(f_new)){
                stateNew = Tstate(state);
                stateNew.floor(f_new);
                if(mBuilding->isMovable(stateNew) && mBuilding->isStair(stateNew)){
                    break;
                }
            }
        }
        return stateNew;
    }
    
    template<class Tstate, class Tinput>
    Tstate moveWithAngle(const Tstate& state, const Tstate& stateNewTentative, const Tinput& input, double angle){
        State stateNew(state);
        double dt= (input.timestamp() - input.previousTimestamp())/(1000.0);
        double v = stateNewTentative.velocity();
        double nV = stateNewTentative.normalVelocity();
        double vx = v * std::cos(angle);
        double vy = v * std::sin(angle);
        stateNew.Location::x(state.Location::x() + vx*dt);
        stateNew.Location::y(state.Location::y() + vy*dt);
        stateNew.velocity(v);
        stateNew.normalVelocity(nV);
        return stateNew;
    }

    template<class Tstate, class Tinput>
    Tstate SystemModelInBuilding<Tstate, Tinput>::moveOnFloor(const Tstate& state, Tinput input){
        if(! mBuilding->isMovable(state)){
            BOOST_THROW_EXCEPTION(LocException("building->isMovable(state) is false"));
        }
        Tstate stateNew(state);
        
        SystemModelVelocityAdjustable* prw = dynamic_cast<SystemModelVelocityAdjustable*>(mSysModel.get());
        auto* sysCtrl = dynamic_cast<SystemModelMovementControllable*>(mSysModel.get());
        if(prw!=NULL){
             // Change field velocity rate
             if(mBuilding->isElevator(state)){
                 prw->velocityRate(mProperty->velocityRateElevator());
             }else if(mBuilding->isStair(state)){
                 prw->velocityRate(mProperty->velocityRateStair());
             }else if(mBuilding->isEscalator(state)){
                 prw->velocityRate(mProperty->velocityRateEscalator());
             }else{
                 prw->velocityRate(mProperty->velocityRateFloor());
             }
        }
        if(sysCtrl!=NULL){
            if(mBuilding->isEscalator(state)){
                sysCtrl->forceMove();
            }
        }
        // Update state
        for(int i=0; i<mProperty->maxTrial() ; i++){
            stateNew = mSysModel->predict(state, input);
            if(mBuilding->checkMovableRoute(state, stateNew)){
                break;
            }else if(i==mProperty->maxTrial()-1){
                stateNew = moveOnFloorRetry(state, stateNew, input);
                if(!mBuilding->checkMovableRoute(state, stateNew)){
                    BOOST_THROW_EXCEPTION(LocException("A route from location (" + static_cast<Location>(state).toString()
                                                        + ") to new location (" + static_cast<Location>(stateNew).toString() + ") is invalid."));
                }
            }
        }
        if(prw!=NULL){
            // Revert field velocity rate
            prw->velocityRate(mProperty->velocityRateFloor());
        }
        if(sysCtrl!=NULL){
            sysCtrl->releaseControl();
        }
        if(! mBuilding->isMovable(stateNew)){
            if (stateNew.weight() != 0){
                BOOST_THROW_EXCEPTION(LocException("stateNew.weight is not 0 even though stateNew is not movable."));
            }
        }
        return stateNew;
    }
    
    template<class Tstate, class Tinput>
    Tstate SystemModelInBuilding<Tstate, Tinput>::moveOnFloorRetry(const Tstate& state, const Tstate& stateNew, Tinput input){
        Tstate stateTmp(stateNew);
        if( mRandomGenerator.nextDouble() < mProperty->wallCrossingAliveRate()){
            double orientation = atan2(stateNew.y() - state.y(), stateNew.x() - state.x());
            double angle = mBuilding->estimateWallAngle(state, stateNew);
            double orientationDiff = Pose::computeOrientationDifference(orientation, angle);
            if(2.0*M_PI < std::abs(orientationDiff)){
                BOOST_THROW_EXCEPTION(LocException("orientationDifference is out of range."));
            }
            if(std::abs(orientationDiff) < mProperty->maxIncidenceAngle()){
                stateTmp = moveWithAngle(state, stateNew, input, angle);
            }
            if(mBuilding->checkMovableRoute(state, stateTmp)){
                return stateTmp;
            }else{
                stateTmp = State(state);
                stateTmp.weight(state.weight() * mProperty->weightDecayRate());
            }
        }
        if(! mBuilding->isMovable(stateTmp)){
            BOOST_THROW_EXCEPTION(LocException());
        }
        return stateTmp;
    }
    
    template<class Tstate, class Tinput>
    Tstate SystemModelInBuilding<Tstate, Tinput>::moveFloorJump(const Tstate& state, Tinput input){
        int f_min = mBuilding->minFloor();
        int f_max = mBuilding->maxFloor();
        Tstate stateNew(state);
        while(true){
            int f_new = f_min + mRandomGenerator.nextInt(f_max - f_min);
            if(mBuilding->isValidFloor(f_new)){
                stateNew = Tstate(state);
                stateNew.floor(f_new);
                if(mBuilding->isMovable(stateNew)){
                    break;
                }
            }
        }
        return stateNew;
    }
    
    template<class Tstate, class Tinput>
    Tstate SystemModelInBuilding<Tstate, Tinput>::predict(Tstate state, Tinput input){
        if(! mBuilding->isMovable(state)){
            BOOST_THROW_EXCEPTION(LocException("building->isMovable(state) == false"));
        }
        try{
            // Jumping move
            if(mRandomGenerator.nextDouble() < mProperty->probabilityFloorJump()){
                Tstate stateTmp = moveFloorJump(state, input);
                return moveOnFloor(stateTmp, input);
            }
            // Standard move
            if(mBuilding->isElevator(state)){
                Tstate stateTmp = moveOnElevator(state, input);
                if(Location::floorDifference(state, stateTmp)==0){
                    return moveOnFloor(stateTmp, input);
                }else{
                    return stateTmp;
                }
            }else if(mBuilding->isEscalator(state)){
                State stateTmp = moveOnEscalator(state, input);
                return moveOnFloor(stateTmp, input);
            }else if(mBuilding->isStair(state)){
                State stateTmp = moveOnStair(state, input);
                return moveOnFloor(stateTmp, input);
            }else{
                return moveOnFloor(state, input);
            }
        }catch(LocException& ex){
            ex << boost::error_info<struct err_info, std::string>("Failed prediction at a given location (" + static_cast<Location>(state).toString() + ")");
            BOOST_THROW_EXCEPTION(ex);
        }
    }

    template<class Tstate, class Tinput>
    std::vector<Tstate> SystemModelInBuilding<Tstate, Tinput>::predict(std::vector<Tstate> states, Tinput input){
        std::vector<Tstate> statesPredicted(states.size());
        mSysModel->startPredictions(states, input);
        for(int i=0; i<states.size(); i++){
            Tstate& st = states.at(i);
            statesPredicted[i] = predict(st, input);
        }
        mSysModel->endPredictions(states, input);
        return statesPredicted;
    }
    
    template<class Tstate, class Tinput>
    void SystemModelInBuilding<Tstate, Tinput>::notifyObservationUpdated(){
        mSysModel->notifyObservationUpdated();
    }
    
    // Explicit instantiation
    template class SystemModelInBuilding<State, SystemModelInput>;
}
