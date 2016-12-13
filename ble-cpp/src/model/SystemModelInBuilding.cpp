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
    SystemModelInBuildingProperty& SystemModelInBuildingProperty::probabilityUpStair(double probabilityUp){
        probabilityUpStair_ = probabilityUp;
        return *this;
    }
    
    SystemModelInBuildingProperty& SystemModelInBuildingProperty::probabilityDownStair(double probabilityDown){
        probabilityDownStair_ = probabilityDown;
        return *this;
    }
    
    SystemModelInBuildingProperty& SystemModelInBuildingProperty::probabilityStayStair(double probabilityStay){
        probabilityStayStair_ = probabilityStay;
        return *this;
    }
    
    SystemModelInBuildingProperty& SystemModelInBuildingProperty::probabilityUpElevator(double probabilityUp){
        probabilityUpElevator_ = probabilityUp;
        return *this;
    }
    
    SystemModelInBuildingProperty& SystemModelInBuildingProperty::probabilityDownElevator(double probabilityDown){
        probabilityDownElevator_ = probabilityDown;
        return *this;
    }
    
    SystemModelInBuildingProperty& SystemModelInBuildingProperty::probabilityStayElevator(double probabilityStay){
        probabilityStayElevator_ = probabilityStay;
        return *this;
    }
    
    SystemModelInBuildingProperty& SystemModelInBuildingProperty::probabilityUpEscalator(double probabilityUp){
        probabilityUpEscalator_ = probabilityUp;
        return *this;
    }
    
    SystemModelInBuildingProperty& SystemModelInBuildingProperty::probabilityDownEscalator(double probabilityDown){
        probabilityDownEscalator_ = probabilityDown;
        return *this;
    }
    
    SystemModelInBuildingProperty& SystemModelInBuildingProperty::probabilityStayEscalator(double probabilityStay){
        probabilityStayEscalator_ = probabilityStay;
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
    
    SystemModelInBuildingProperty& SystemModelInBuildingProperty::relativeVelocityEscalator(double relativeVelocityEscalator){
        relativeVelocityEscalator_ = relativeVelocityEscalator;
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
    
    double SystemModelInBuildingProperty::probabilityUpStair() const{
        return probabilityUpStair_;
    }
    
    double SystemModelInBuildingProperty::probabilityDownStair() const{
        return probabilityDownStair_;
    }
    
    double SystemModelInBuildingProperty::probabilityStayStair() const{
        return probabilityStayStair_;
    }
    
    double SystemModelInBuildingProperty::probabilityUpElevator() const{
        return probabilityUpElevator_;
    }
    
    double SystemModelInBuildingProperty::probabilityDownElevator() const{
        return probabilityDownElevator_;
    }
    
    double SystemModelInBuildingProperty::probabilityStayElevator() const{
        return probabilityStayElevator_;
    }
    
    double SystemModelInBuildingProperty::probabilityUpEscalator() const{
        return probabilityUpEscalator_;
    }
    
    double SystemModelInBuildingProperty::probabilityDownEscalator() const{
        return probabilityDownEscalator_;
    }
    
    double SystemModelInBuildingProperty::probabilityStayEscalator() const{
        return probabilityStayEscalator_;
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
    
    double SystemModelInBuildingProperty::relativeVelocityEscalator() const{
        return relativeVelocityEscalator_;
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
        int f_current = std::round(state.floor());
        
        // pUp and pDown are currently not used.
        double pStay = mProperty->probabilityStayElevator();
        
        Tstate stateNew(state);
        if(f_min == f_max){
            return stateNew;
        }
        while(true){
            double p = mRandomGenerator.nextDouble();
            if(p<=pStay){
                break;
            }else{
                int f_new = f_current;
                while(true){
                    f_new = f_min + mRandomGenerator.nextInt(f_max - f_min);
                    if(f_new != f_current){
                        break;
                    }
                }
                if(mBuilding->isValidFloor(f_new)){
                    stateNew = Tstate(state);
                    stateNew.floor(f_new);
                    if(mBuilding->isMovable(stateNew) && mBuilding->isElevator(stateNew)){
                        break;
                    }
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
        
        double pUp = mProperty->probabilityUpEscalator();
        double pDown = mProperty->probabilityDownEscalator();
        double pStay = mProperty->probabilityStayEscalator();
        
        if(f==f_min){
            pDown = 0;
            pUp = mProperty->probabilityDownEscalator()/(mProperty->probabilityDownEscalator() + mProperty->probabilityStayEscalator());
            pStay = 1 - pUp;
        }else if(f==f_max){
            pDown = mProperty->probabilityUpEscalator()/(mProperty->probabilityUpEscalator() + mProperty->probabilityStayEscalator());
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
        
        double pUp = mProperty->probabilityUpStair();
        double pDown = mProperty->probabilityDownStair();
        double pStay = mProperty->probabilityStayStair();
        
        if(f==f_min){
            pDown = 0;
            pUp = mProperty->probabilityDownStair()/(mProperty->probabilityDownStair() + mProperty->probabilityStayStair());
            pStay = 1 - pUp;
        }else if(f==f_max){
            pDown = mProperty->probabilityUpStair()/(mProperty->probabilityUpStair() + mProperty->probabilityStayStair());
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
        
        auto sysVelAdj = std::dynamic_pointer_cast<SystemModelVelocityAdjustable>(mSysModel);
        auto sysCtrl = std::dynamic_pointer_cast<SystemModelMovementControllable>(mSysModel);
        if(sysVelAdj!=NULL){
             // Change field velocity
             if(mBuilding->isElevator(state)){
                 sysVelAdj->velocityRate(mProperty->velocityRateElevator());
             }else if(mBuilding->isStair(state)){
                 sysVelAdj->velocityRate(mProperty->velocityRateStair());
             }else if(mBuilding->isEscalatorGroup(state)){
                 sysVelAdj->velocityRate(mProperty->velocityRateEscalator());
                 sysVelAdj->relativeVelocity(mProperty->relativeVelocityEscalator());
             }else{
                 sysVelAdj->velocityRate(mProperty->velocityRateFloor());
             }
        }
        if(sysCtrl!=NULL){
            if(mBuilding->isEscalatorGroup(state)){
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
        if(sysVelAdj!=NULL){
            // Revert field velocity
            sysVelAdj->velocityRate(mProperty->velocityRateFloor());
            sysVelAdj->relativeVelocity(0.0);
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
            }else if(mBuilding->isEscalator(state)){ // escalator move is not allowed on escalator end
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
